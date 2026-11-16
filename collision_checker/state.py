"""Selection semantics, separate from immutable sample statistics."""

from __future__ import annotations

import copy

from .analysis import DEFAULTS, validate_settings
from .model import finite


class Session:
    def __init__(self, robot, revision):
        self.robot, self.revision = robot, revision
        self.settings = copy.deepcopy(DEFAULTS)
        self.choices = {
            key: dict(manual=None, lock=None, min_distance=None, topk=False)
            for key in robot.pairs
        }
        self.stats, self.elapsed_seconds, self.report = {}, None, None
        self.stale, self.generation, self.undo = True, 0, None

    def invalidate(self):
        self.stale = True
        self.generation += 1
        self.report = None

    def update_settings(self, patch):
        new = {**self.settings, **patch}
        new["rules"] = {**self.settings["rules"], **patch.get("rules", {})}
        new = validate_settings(self.robot, new, require_ranges=False)
        analysis_keys = {
            "samples",
            "seed",
            "always_threshold",
            "default_min_distance",
            "excluded",
            "ranges",
        }
        if any(new.get(k) != self.settings.get(k) for k in analysis_keys):
            self.invalidate()
        self.settings = new
        self.generation += 1
        self.report = None
        self.undo = None

    def update_pair(self, key, patch):
        if key not in self.choices:
            raise ValueError("未知碰撞对")
        choice = self.choices[key]
        if (
            "manual" in patch
            and patch["manual"] is not None
            and type(patch["manual"]) is not bool
        ):
            raise ValueError("manual 必须为布尔或 null")
        if "lock" in patch and patch["lock"] not in (None, "keep", "exclude"):
            raise ValueError("无效锁定类型")
        if "min_distance" in patch and patch["min_distance"] is not None:
            patch["min_distance"] = finite(patch["min_distance"], "min_distance", True)
        if "min_distance" in patch and patch["min_distance"] != choice["min_distance"]:
            self.invalidate()
        choice.update({k: v for k, v in patch.items() if k in choice})
        if "manual" in patch or "lock" in patch:
            choice["topk"] = False
        self.generation += 1
        self.report = None
        self.undo = None

    def reason(self, key):
        p, c = self.robot.pairs[key], self.choices[key]
        if {p["link1"], p["link2"]} & set(self.settings["excluded"]):
            return "Excluded"
        if c["lock"] == "exclude":
            return "User"
        if c["lock"] == "keep":
            return None
        if c["topk"]:
            return "TopK"
        if c["manual"] is not None:
            return None if c["manual"] else "User"
        return next(
            (
                r
                for r in self.stats.get(key, {}).get("classifications", [])
                if self.settings["rules"].get(r)
            ),
            None,
        )

    def selected(self):
        return [key for key in self.choices if self.reason(key) is None]

    def topk(self, k, apply=False):
        if self.stale:
            raise ValueError("采样统计待更新，请先重新分析")
        if not isinstance(k, int) or k < 0:
            raise ValueError("K 必须是非负整数")
        candidates = self.selected()
        kept = [key for key in candidates if self.choices[key]["lock"] == "keep"]
        ranking = sorted(
            (key for key in candidates if key not in kept),
            key=lambda key: (
                -self.stats[key]["near_rate"],
                -self.stats[key]["collision_rate"],
                self.stats[key]["min_distance"],
                self.robot.pairs[key]["link1"],
                self.robot.pairs[key]["link2"],
            ),
        )
        kept += ranking[: max(0, k - len(kept))]
        kept_set = set(kept)
        total_hits = sum(self.stats[key]["count"] for key in candidates)
        report = dict(
            k=k,
            kept=kept,
            removed=[key for key in candidates if key not in kept_set],
            before=len(candidates),
            after=len(kept),
            exceeds_k=len(kept) > k,
            collision_event_retention=None
            if not total_hits
            else sum(self.stats[key]["count"] for key in kept) / total_hits,
        )
        if apply:
            self.undo = copy.deepcopy(self.choices)
            for key in report["removed"]:
                self.choices[key]["topk"] = True
            self.generation += 1
            self.report = None
        return report

    def accept_analysis(self, result):
        self.stats, self.elapsed_seconds = result["stats"], result["elapsed_seconds"]
        self.stale = False
        for c in self.choices.values():
            c["topk"] = False
        self.undo = None

    def public(self):
        pairs = []
        for key, p in self.robot.pairs.items():
            c = self.choices[key]
            pairs.append(
                {
                    **p,
                    **c,
                    "reason": self.reason(key),
                    "enabled": self.reason(key) is None,
                    "effective_min_distance": c["min_distance"]
                    if c["min_distance"] is not None
                    else self.settings["default_min_distance"],
                    "stats": self.stats.get(key),
                }
            )
        return dict(
            model=self.robot.public(self.revision),
            settings=self.settings,
            pairs=pairs,
            stale=self.stale,
            generation=self.generation,
            elapsed_seconds=self.elapsed_seconds,
            benchmark=self.report,
            can_undo=self.undo is not None,
        )

    def project(self):
        return dict(
            version=1,
            source=dict(
                path=self.robot.path,
                packages=self.robot.packages,
                xacro_args=self.robot.xacro_args,
            ),
            fingerprint=self.robot.fingerprint,
            versions=self.robot.public(self.revision)["versions"],
            settings=self.settings,
            choices=self.choices,
            stats=self.stats,
            stale=self.stale,
            elapsed_seconds=self.elapsed_seconds,
            benchmark=self.report,
            pose=self.robot.pose,
        )
