"""One authoritative URDF/Pinocchio geometry model, with stable object indices."""

from __future__ import annotations

import hashlib
import itertools
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import coal
import pinocchio as pin


def finite(value, name="value", nonnegative=False):
    if isinstance(value, bool):
        raise ValueError(f"{name} 必须是数值")
    try:
        value = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} 必须是数值") from exc
    if not math.isfinite(value) or (nonnegative and value < 0):
        raise ValueError(f"{name} 必须是有限{'非负' if nonnegative else ''}数")
    return value


class Robot:
    def __init__(self, path, packages=None, xacro_args=None):
        self.path = str(Path(path).expanduser().resolve())
        self.packages = {
            k: str(Path(v).expanduser().resolve()) for k, v in (packages or {}).items()
        }
        self.xacro_args = xacro_args or {}
        source = Path(self.path)
        if source.is_dir():
            candidates = sorted(source.rglob("*.urdf")) + sorted(
                source.rglob("*.xacro")
            )
            if len(candidates) != 1:
                raise ValueError(
                    "目录中需要恰好一个 URDF/Xacro，否则请填写具体文件路径"
                )
            source = candidates[0]
            self.path = str(source)
        if source.suffix == ".xacro":
            try:
                import xacro
            except ImportError as exc:
                raise ValueError("请安装可选依赖：uv pip install '.[xacro]'") from exc
            xml = xacro.process_file(str(source), mappings=self.xacro_args).toxml()
        else:
            xml = source.read_text()
        try:
            root = ET.fromstring(xml)
        except ET.ParseError as exc:
            raise ValueError(f"URDF XML 格式错误：{exc}") from exc
        if root.tag != "robot":
            raise ValueError("文件必须包含 robot 根节点")
        self.name = root.get("name", "robot")
        for elem in root.findall(".//origin") + root.findall(".//axis"):
            for attr in ("xyz", "rpy"):
                if elem.get(attr) is None:
                    continue
                values = [
                    finite(v, f"{elem.tag}.{attr}") for v in elem.get(attr).split()
                ]
                if len(values) != 3 or (
                    elem.tag == "axis" and sum(v * v for v in values) == 0
                ):
                    raise ValueError(f"无效的三维向量：{elem.tag}.{attr}")
        for geometry in root.findall(".//geometry"):
            if len(geometry) != 1 or geometry[0].tag not in {
                "sphere",
                "box",
                "cylinder",
                "mesh",
            }:
                raise ValueError("几何体仅支持 sphere、box、cylinder、mesh")
            shape = geometry[0]
            attrs = {
                "sphere": ["radius"],
                "box": ["size"],
                "cylinder": ["radius", "length"],
                "mesh": [],
            }[shape.tag]
            for attr in attrs:
                values = [
                    finite(v, f"{shape.tag}.{attr}")
                    for v in shape.get(attr, "").split()
                ]
                if len(values) != (3 if attr == "size" else 1) or any(
                    v <= 0 for v in values
                ):
                    raise ValueError(f"无效的几何尺寸：{shape.tag}.{attr}")
            if shape.tag == "mesh":
                values = [
                    finite(v, "mesh.scale") for v in shape.get("scale", "1 1 1").split()
                ]
                if len(values) != 3 or any(v <= 0 for v in values):
                    raise ValueError("mesh.scale 必须是三个正数")
        self.assets = {}
        digest = hashlib.sha256(xml.encode())
        # Resolve once for both render and compute. No fuzzy basename matching.
        resources = root.findall(".//mesh") + root.findall(".//texture")
        for mesh in resources:
            filename = mesh.get("filename", "")
            if filename.startswith("package://"):
                package, _, rest = filename[10:].partition("/")
                if package not in self.packages:
                    raise ValueError(f"请配置 package://{package} 的路径")
                asset = Path(self.packages[package]) / rest
            elif filename.startswith("file://"):
                asset = Path(filename[7:])
            else:
                asset = source.parent / filename
            asset = asset.resolve()
            if not asset.is_file():
                raise ValueError(f"找不到几何资源：{filename} → {asset}")
            digest.update(str(asset).encode())
            digest.update(asset.read_bytes())
            asset_id = hashlib.sha256(str(asset).encode()).hexdigest()[:24]
            self.assets[asset_id] = str(asset)
            mesh.set("filename", str(asset))
        # Collada embeds references to image files. Admit them explicitly too;
        # the file endpoint never exposes arbitrary neighboring files.
        for value in list(self.assets.values()):
            asset = Path(value)
            if asset.suffix.lower() != ".dae":
                continue
            tree = ET.parse(asset)
            for elem in tree.findall(".//{*}image/{*}init_from"):
                filename = (elem.text or "").strip()
                if not filename or filename.startswith("data:"):
                    continue
                from urllib.parse import unquote

                image = (
                    asset.parent / unquote(filename.removeprefix("file://"))
                ).resolve()
                if not image.is_file():
                    raise ValueError(f"找不到 Collada 纹理：{image}")
                digest.update(str(image).encode())
                digest.update(image.read_bytes())
                self.assets[hashlib.sha256(str(image).encode()).hexdigest()[:24]] = str(
                    image
                )
        self.xml = ET.tostring(root, encoding="unicode")
        self.fingerprint = digest.hexdigest()
        self.links = [e.get("name") for e in root.findall("link")]
        if any(not name for name in self.links):
            raise ValueError("link 必须有非空名称")
        if len(set(self.links)) != len(self.links):
            raise ValueError("重复的 link 名称")
        self.joints = []
        self.parents = {}
        for elem in root.findall("joint"):
            if (
                not elem.get("name")
                or elem.find("parent") is None
                or elem.find("child") is None
            ):
                raise ValueError("joint 必须有名称、parent 和 child")
            kind = elem.get("type")
            if kind not in {"fixed", "revolute", "continuous", "prismatic"}:
                raise ValueError(f"不支持的关节类型：{elem.get('name')} ({kind})")
            parent, child = (
                elem.find("parent").get("link"),
                elem.find("child").get("link"),
            )
            if (
                parent not in self.links
                or child not in self.links
                or child in self.parents
            ):
                raise ValueError("关节树中存在未知 link 或多个父节点")
            self.parents[child] = parent
            limit, mimic = elem.find("limit"), elem.find("mimic")
            lower = -math.pi if kind == "continuous" else self._limit(limit, "lower")
            upper = math.pi if kind == "continuous" else self._limit(limit, "upper")
            if lower is not None and upper is not None and lower > upper:
                raise ValueError(f"关节限位颠倒：{elem.get('name')}")
            self.joints.append(
                dict(
                    name=elem.get("name"),
                    type=kind,
                    parent=parent,
                    child=child,
                    lower=lower,
                    upper=upper,
                    mimic=None
                    if mimic is None
                    else dict(
                        joint=mimic.get("joint"),
                        multiplier=finite(mimic.get("multiplier", 1)),
                        offset=finite(mimic.get("offset", 0)),
                    ),
                )
            )
        names = [j["name"] for j in self.joints]
        if len(names) != len(set(names)):
            raise ValueError("重复的 joint 名称")
        self.by_joint = {j["name"]: j for j in self.joints}
        self._check_tree()
        self.feasible_bounds = self._feasible_bounds()
        # Keep mimic joints independent in Pinocchio and explicitly enforce URDF mimic.
        self.model = pin.buildModelFromXML(self.xml)
        self.geometry = pin.buildGeomFromUrdfString(self.model, self.xml, pin.COLLISION)
        self.objects = []
        self.by_link = {name: [] for name in self.links}
        for i, obj in enumerate(self.geometry.geometryObjects):
            link = self.model.frames[obj.parentFrame].name
            if link not in self.by_link:
                raise ValueError(f"几何体所属 frame 不是 URDF link：{link}")
            self.by_link[link].append(i)
            self.objects.append(dict(index=i, name=obj.name, link=link))
        for link in root.findall("link"):
            if len(link.findall("collision")) != len(self.by_link[link.get("name")]):
                raise ValueError(f"collision 几何解析数量不一致：{link.get('name')}")
        self.collision_links = sorted(k for k, v in self.by_link.items() if v)
        self.pairs = {}
        for a, b in itertools.combinations(self.collision_links, 2):
            key = self.pair_key(a, b)
            self.pairs[key] = dict(
                id=key,
                link1=a,
                link2=b,
                objects=[
                    list(p) for p in itertools.product(self.by_link[a], self.by_link[b])
                ],
                adjacent=self._adjacent(a, b),
            )
        self.data = self.model.createData()
        self.geom_data = self.geometry.createData()
        self.default_pose = {
            name: min(max(0.0, bounds[0]), bounds[1])
            for name, bounds in self.feasible_bounds.items()
        }
        self.pose = self.resolve_pose(self.default_pose)
        self.place(self.pose)

    @staticmethod
    def _limit(limit, key):
        if limit is None or limit.get(key) is None:
            return None
        value = float(limit.get(key))
        return value if math.isfinite(value) else None

    @staticmethod
    def pair_key(a, b):
        # Length-prefixing makes IDs unambiguous even with punctuation in names.
        a, b = sorted([a, b])
        return f"{len(a)}:{a}{b}"

    def _check_tree(self):
        roots = set(self.links) - set(self.parents)
        if len(roots) != 1:
            raise ValueError("URDF 必须是单根树")
        for link in self.links:
            seen = set()
            while link in self.parents:
                if link in seen:
                    raise ValueError("URDF 存在闭链")
                seen.add(link)
                link = self.parents[link]
        for joint in self.joints:
            seen = set()
            while joint["mimic"]:
                if (
                    joint["name"] in seen
                    or joint["mimic"]["joint"] not in self.by_joint
                ):
                    raise ValueError("mimic 引用不存在或构成循环")
                seen.add(joint["name"])
                joint = self.by_joint[joint["mimic"]["joint"]]

    def _adjacent(self, a, b):
        # Traverse undirected tree, but do not pass another collision-bearing link.
        todo, seen = [a], {a}
        while todo:
            link = todo.pop()
            neighbors = [c for c, p in self.parents.items() if p == link]
            if link in self.parents:
                neighbors.append(self.parents[link])
            for n in neighbors:
                if n == b:
                    return True
                if n not in seen and not self.by_link[n]:
                    seen.add(n)
                    todo.append(n)
        return False

    def _feasible_bounds(self):
        """URDF mimic chains are affine: all dependent limits constrain a driver interval."""
        bounds = {
            j["name"]: [-math.inf, math.inf]
            for j in self.joints
            if j["type"] != "fixed" and not j["mimic"]
        }

        def expression(j):
            if not j["mimic"]:
                return j["name"], 1.0, 0.0
            m = j["mimic"]
            name, a, b = expression(self.by_joint[m["joint"]])
            return name, a * m["multiplier"], b * m["multiplier"] + m["offset"]

        for joint in self.joints:
            if joint["type"] == "fixed" or joint["type"] == "continuous":
                continue
            name, a, b = expression(joint)
            if name not in bounds:
                raise ValueError("活动 mimic 关节不能由 fixed 关节驱动")
            lo = joint["lower"] if joint["lower"] is not None else -math.inf
            hi = joint["upper"] if joint["upper"] is not None else math.inf
            if a == 0:
                if not lo <= b <= hi:
                    raise ValueError(f"mimic 常量值超出限位：{joint['name']}")
                continue
            left, right = sorted([(lo - b) / a, (hi - b) / a])
            bounds[name][0] = max(bounds[name][0], left)
            bounds[name][1] = min(bounds[name][1], right)
            if bounds[name][0] > bounds[name][1]:
                raise ValueError(f"mimic 限位没有可行解：{name}")
        return bounds

    def resolve_pose(self, values):
        unknown = set(values) - set(self.by_joint)
        if unknown:
            raise ValueError(f"未知关节：{unknown}")
        pose = {}

        def visit(name):
            if name in pose:
                return pose[name]
            j = self.by_joint[name]
            if j["type"] == "fixed":
                value = 0.0
            elif j["mimic"]:
                m = j["mimic"]
                value = visit(m["joint"]) * m["multiplier"] + m["offset"]
            else:
                value = finite(values.get(name, self.default_pose.get(name, 0)), name)
            if j["type"] != "continuous":
                if j["lower"] is not None and value < j["lower"] - 1e-9:
                    raise ValueError(f"关节低于限位：{name}")
                if j["upper"] is not None and value > j["upper"] + 1e-9:
                    raise ValueError(f"关节高于限位：{name}")
            pose[name] = value
            return value

        for j in self.joints:
            visit(j["name"])
        return pose

    def place(self, pose):
        self.pose = self.resolve_pose(pose)
        q = pin.neutral(self.model)
        for j in self.joints:
            if j["type"] == "fixed":
                continue
            jid = self.model.getJointId(j["name"])
            joint = self.model.joints[jid]
            value = self.pose[j["name"]]
            if joint.nq == 2:
                q[joint.idx_q : joint.idx_q + 2] = [math.cos(value), math.sin(value)]
            elif joint.nq == 1:
                q[joint.idx_q] = value
            else:
                raise ValueError(f"不支持的配置维度：{j['name']}")
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.updateGeometryPlacements(
            self.model, self.data, self.geometry, self.geom_data
        )

    def evaluate(self, pair, mode="both"):
        hit, distance = False, math.inf
        for i, j in pair["objects"]:
            a, b = self.geometry.geometryObjects[i], self.geometry.geometryObjects[j]
            ta, tb = self.geom_data.oMg[i], self.geom_data.oMg[j]
            xa, xb = (
                coal.Transform3s(ta.rotation, ta.translation),
                coal.Transform3s(tb.rotation, tb.translation),
            )
            if mode != "distance":
                result = coal.CollisionResult()
                coal.collide(
                    a.geometry, xa, b.geometry, xb, coal.CollisionRequest(), result
                )
                hit = result.isCollision() or hit
            if mode != "collision":
                result = coal.DistanceResult()
                d = coal.distance(
                    a.geometry, xa, b.geometry, xb, coal.DistanceRequest(), result
                )
                if not math.isfinite(d):
                    raise ValueError(f"无法计算有限距离：{a.name}, {b.name}")
                distance = min(distance, max(0.0, float(d)))
        return dict(collision=hit, distance=None if mode == "collision" else distance)

    def browser_xml(self, revision):
        root = ET.fromstring(self.xml)
        for link in root.findall("link"):
            for elem, index in zip(
                link.findall("collision"), self.by_link[link.get("name")]
            ):
                elem.set("name", f"__cc_object_{index}")
        inverse = {v: k for k, v in self.assets.items()}
        for mesh in root.findall(".//mesh") + root.findall(".//texture"):
            from urllib.parse import quote

            filename = mesh.get("filename")
            mesh.set(
                "filename",
                f"/api/assets/{revision}/{inverse[filename]}/{quote(Path(filename).name)}",
            )
        return ET.tostring(root, encoding="unicode")

    def browser_collada(self, asset_id, revision):
        """Rewrite only image references, leaving original on-disk assets intact."""
        from urllib.parse import quote, unquote

        asset = Path(self.assets[asset_id])
        root = ET.parse(asset).getroot()
        if root.tag.startswith("{"):
            ET.register_namespace("", root.tag.split("}")[0][1:])
        inverse = {v: k for k, v in self.assets.items()}
        for elem in root.findall(".//{*}image/{*}init_from"):
            filename = (elem.text or "").strip()
            if not filename or filename.startswith("data:"):
                continue
            image = (asset.parent / unquote(filename.removeprefix("file://"))).resolve()
            elem.text = (
                f"/api/assets/{revision}/{inverse[str(image)]}/{quote(image.name)}"
            )
        return ET.tostring(root, encoding="unicode")

    def public(self, revision):
        return dict(
            name=self.name,
            path=self.path,
            packages=self.packages,
            xacro_args=self.xacro_args,
            fingerprint=self.fingerprint,
            revision=revision,
            links=self.links,
            collision_links=self.collision_links,
            joints=self.joints,
            objects=self.objects,
            default_pose=self.default_pose,
            pose=self.pose,
            xml=self.browser_xml(revision),
            valid_bounds={
                name: dict(
                    lower=lo if math.isfinite(lo) else None,
                    upper=hi if math.isfinite(hi) else None,
                )
                for name, (lo, hi) in self.feasible_bounds.items()
            },
            versions=dict(pinocchio=pin.__version__, coal=coal.__version__),
        )
