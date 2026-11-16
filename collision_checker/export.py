"""OCS2 property-tree INFO, SRDF, and versioned YAML exports."""

from __future__ import annotations

import json
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .model import finite


class InfoDistanceConfirmation(ValueError):
    pass


def atomic_text(path, content):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as stream:
            temporary = stream.name
            stream.write(content)
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def export_files(session, directory, acknowledge_info_global_distance=False):
    # Validate numerical settings here even if the session has not been sampled.
    s = session.settings
    distance = finite(s["default_min_distance"], "default_min_distance", True)
    mu, delta = finite(s["mu"], "mu", True), finite(s["delta"], "delta", True)
    if mu == 0 or delta == 0:
        raise ValueError("mu 和 delta 必须为正数")
    keys = session.selected()
    if (
        any(
            session.choices[key]["min_distance"] is not None
            and session.choices[key]["min_distance"] != distance
            for key in keys
        )
        and not acknowledge_info_global_distance
    ):
        raise InfoDistanceConfirmation(
            "OCS2 INFO 仅支持全局 minimumDistance；逐对距离仅保存在 YAML。请明确确认后导出。"
        )
    r = session.robot
    metadata = dict(
        fingerprint=r.fingerprint,
        versions=r.public(session.revision)["versions"],
        object_index_basis="Pinocchio full collision geometry model, zero-based",
        objects=r.objects,
    )
    pairs, raw_pairs = [], []
    for key in keys:
        p, c = r.pairs[key], session.choices[key]
        item = dict(link1=p["link1"], link2=p["link2"], enabled=True)
        if c["min_distance"] is not None:
            item["min_distance"] = c["min_distance"]
        item["collisionObjectPairs"] = [
            dict(
                object1=i,
                object2=j,
                object1_name=r.objects[i]["name"],
                object2_name=r.objects[j]["name"],
            )
            for i, j in p["objects"]
        ]
        pairs.append(item)
        raw_pairs.extend(p["objects"])
    document = dict(
        version=1,
        metadata=metadata,
        self_collision=dict(
            enabled=s["enabled"],
            default_min_distance=distance,
            collisionLinkPairs=pairs,
        ),
    )
    xml = ET.Element("robot", name=r.name)
    xml.append(ET.Comment(f" model fingerprint: {r.fingerprint} "))
    for key, p in r.pairs.items():
        if key not in keys:
            ET.SubElement(
                xml,
                "disable_collisions",
                link1=p["link1"],
                link2=p["link2"],
                reason=session.reason(key),
            )
    ET.indent(xml)
    # Only OCS2-recognized active keys. Mapping metadata is in semicolon comments.
    lines = [
        f"; model fingerprint: {r.fingerprint}",
        "; Object indices require the same URDF and Pinocchio geometry ordering.",
        "; Merge this selfCollision block into task.info, replacing its previous block.",
    ]
    for obj in r.objects:
        lines.append(
            f"; object {obj['index']}: {json.dumps(obj['name'], ensure_ascii=True)} / link {json.dumps(obj['link'], ensure_ascii=True)}"
        )
    lines += [
        "selfCollision",
        "{",
        f"  activate {'true' if s['enabled'] else 'false'}",
        "  collisionObjectPairs",
        "  {",
    ]
    for index, (i, j) in enumerate(raw_pairs):
        lines.append(f'    [{index}] "{i}, {j}"')
    lines += ["  }", "  collisionLinkPairs", "  {"]
    for index, p in enumerate(pairs):
        # OCS2 LoadStdVectorOfPair splits strings at comma; reject ambiguous names.
        for name in (p["link1"], p["link2"]):
            if any(char in name for char in ',"\\') or any(
                char.isspace() for char in name
            ):
                raise ValueError(f"link 名称无法无损表示为 OCS2 配对字符串：{name!r}")
        lines.append(f'    [{index}] "{p["link1"]}, {p["link2"]}"')
    lines += [
        "  }",
        f"  minimumDistance {distance:.17g}",
        f"  mu {mu:.17g}",
        f"  delta {delta:.17g}",
        "}",
        "",
    ]
    contents = {
        "disabled_collisions.srdf": '<?xml version="1.0" encoding="UTF-8"?>\n'
        + ET.tostring(xml, encoding="unicode")
        + "\n",
        "self_collision.info": "\n".join(lines),
        "self_collision.yaml": yaml.safe_dump(
            document, allow_unicode=True, sort_keys=False
        ),
    }
    output = Path(directory).expanduser().resolve()
    for name, content in contents.items():
        atomic_text(output / name, content)
    return dict(
        files=[str(output / name) for name in contents],
        link_pairs=len(keys),
        object_pairs=len(raw_pairs),
        stale=session.stale,
    )


def read_enabled_yaml(path):
    """Reference consumer: expand enabled YAML entries with effective distances."""
    document = yaml.safe_load(Path(path).read_text())
    if document.get("version") != 1:
        raise ValueError("不支持的 YAML 版本")
    config = document["self_collision"]
    default = finite(
        config.get("default_min_distance", 0.05), "default_min_distance", True
    )
    if type(config.get("enabled", True)) is not bool:
        raise ValueError("enabled 必须为布尔值")
    result = []
    for pair in config.get("collisionLinkPairs", []):
        if type(pair.get("enabled", True)) is not bool:
            raise ValueError("enabled 必须为布尔值")
        distance = finite(pair.get("min_distance", default), "min_distance", True)
        if config.get("enabled", True) and pair.get("enabled", True):
            for obj in pair["collisionObjectPairs"]:
                result.append(
                    {
                        **obj,
                        "link1": pair["link1"],
                        "link2": pair["link2"],
                        "min_distance": distance,
                    }
                )
    return result
