"""Deterministic analysis and single-process wall-clock benchmarks."""

from __future__ import annotations

import time

import numpy as np

from .model import Robot, finite

DEFAULTS = dict(
    samples=10000,
    seed=42,
    always_threshold=0.95,
    default_min_distance=0.05,
    enabled=True,
    mu=0.01,
    delta=0.001,
    excluded=[],
    ranges={},
    rules=dict(Adjacent=True, Default=True, Always=True, Never=True),
)


def validate_settings(robot, settings, require_ranges=True):
    if set(settings) - set(DEFAULTS):
        raise ValueError("未知分析设置")
    s = {**DEFAULTS, **settings}
    if not isinstance(s["rules"], dict) or set(s["rules"]) - set(DEFAULTS["rules"]):
        raise ValueError("未知自动禁用规则")
    s["rules"] = {**DEFAULTS["rules"], **s["rules"]}
    if type(s["enabled"]) is not bool or any(
        type(v) is not bool for v in s["rules"].values()
    ):
        raise ValueError("enabled 和禁用规则必须是布尔值")
    if type(s["samples"]) is not int or not 1 <= s["samples"] <= 1000000:
        raise ValueError("采样次数必须在 1 到 1000000 之间")
    if type(s["seed"]) is not int or not 0 <= s["seed"] < 2**32:
        raise ValueError("随机种子必须是 32 位非负整数")
    s["always_threshold"] = finite(s["always_threshold"], "Always 阈值")
    if not 0 < s["always_threshold"] <= 1:
        raise ValueError("Always 阈值必须在 (0, 1] 内")
    for key in ("default_min_distance", "mu", "delta"):
        s[key] = finite(s[key], key, True)
    if s["mu"] == 0 or s["delta"] == 0:
        raise ValueError("mu 和 delta 必须为正数")
    if not isinstance(s["excluded"], list) or any(
        not isinstance(n, str) for n in s["excluded"]
    ):
        raise ValueError("excluded 必须是 link 名称列表")
    if set(s["excluded"]) - set(robot.links):
        raise ValueError("排除列表包含未知 link")
    if not isinstance(s["ranges"], dict) or set(s["ranges"]) - set(robot.by_joint):
        raise ValueError("采样设置包含未知关节")
    for j in robot.joints:
        if j["type"] == "fixed" or j["mimic"]:
            continue
        r = s["ranges"].get(j["name"], {})
        if not isinstance(r, dict) or set(r) - {"lower", "upper", "locked", "value"}:
            raise ValueError(f"无效的关节采样设置：{j['name']}")
        if type(r.get("locked", False)) is not bool:
            raise ValueError("locked 必须为布尔值")
        if r.get("locked", False):
            robot.resolve_pose(
                {
                    **robot.default_pose,
                    j["name"]: finite(r.get("value", robot.default_pose[j["name"]])),
                }
            )
            continue
        lo, hi = r.get("lower", j["lower"]), r.get("upper", j["upper"])
        # Missing URDF limits are allowed at model load, but block analysis.
        if lo is None or hi is None:
            if not require_ranges:
                continue
            raise ValueError(f"请设置关节采样范围或锁定：{j['name']}")
        lo, hi = finite(lo), finite(hi)
        if lo > hi:
            raise ValueError(f"关节采样下限大于上限：{j['name']}")
        if j["type"] != "continuous" and (
            (j["lower"] is not None and lo < j["lower"])
            or (j["upper"] is not None and hi > j["upper"])
        ):
            raise ValueError(f"采样范围超出 URDF 限位：{j['name']}")
        valid_lo, valid_hi = robot.feasible_bounds[j["name"]]
        if max(lo, valid_lo) > min(hi, valid_hi):
            raise ValueError(f"采样范围无法满足 mimic 限位：{j['name']}")
    return s


def sample_pose(robot, settings, rng):
    # Intersect driver ranges with affine mimic constraints, never reject collisions.
    values = {}
    for j in robot.joints:
        if j["type"] == "fixed" or j["mimic"]:
            continue
        r = settings["ranges"].get(j["name"], {})
        valid_lo, valid_hi = robot.feasible_bounds[j["name"]]
        values[j["name"]] = (
            r.get("value", robot.default_pose[j["name"]])
            if r.get("locked")
            else float(
                rng.uniform(
                    max(r.get("lower", j["lower"]), valid_lo),
                    min(r.get("upper", j["upper"]), valid_hi),
                )
            )
        )
    return robot.resolve_pose(values)


def analyze(
    robot, settings, overrides, progress=lambda *_: None, cancelled=lambda: False
):
    settings = validate_settings(robot, settings)
    start = time.perf_counter()
    active = [
        p
        for p in robot.pairs.values()
        if not {p["link1"], p["link2"]} & set(settings["excluded"])
    ]
    robot.place(robot.default_pose)
    stats = {}
    for p in active:
        initial = robot.evaluate(p)
        stats[p["id"]] = dict(
            count=0,
            near_count=0,
            min_distance=None,
            collision_pose=None,
            closest_pose=None,
            default_collision=initial["collision"],
        )
    rng = np.random.default_rng(settings["seed"])
    n = settings["samples"]
    for index in range(n):
        if cancelled():
            return None
        pose = sample_pose(robot, settings, rng)
        robot.place(pose)
        for p in active:
            result, stat = robot.evaluate(p), stats[p["id"]]
            hit, distance = result["collision"], result["distance"]
            threshold = overrides.get(p["id"], settings["default_min_distance"])
            stat["count"] += int(hit)
            stat["near_count"] += int(hit or distance < threshold)
            if hit and stat["collision_pose"] is None:
                stat["collision_pose"] = dict(pose)
            if stat["min_distance"] is None or distance < stat["min_distance"]:
                stat["min_distance"], stat["closest_pose"] = distance, dict(pose)
        if index % max(1, n // 100) == 0 or index == n - 1:
            progress((index + 1) / n, f"采样 {index + 1}/{n}")
    for p in active:
        stat = stats[p["id"]]
        stat["collision_rate"] = stat["count"] / n
        stat["near_rate"] = stat["near_count"] / n
        stat["samples"] = n
        stat["classifications"] = [
            reason
            for reason, test in (
                ("Adjacent", p["adjacent"]),
                ("Default", stat["default_collision"]),
                ("Always", stat["collision_rate"] >= settings["always_threshold"]),
                ("Never", stat["count"] == 0),
            )
            if test
        ]
    return dict(stats=stats, elapsed_seconds=time.perf_counter() - start)


def benchmark(
    robot,
    settings,
    selected,
    samples=200,
    progress=lambda *_: None,
    cancelled=lambda: False,
):
    settings = validate_settings(robot, settings)
    rng = np.random.default_rng((settings["seed"] + 100003) % 2**32)
    poses = [sample_pose(robot, settings, rng) for _ in range(samples)]
    all_pairs = [
        p
        for p in robot.pairs.values()
        if not {p["link1"], p["link2"]} & set(settings["excluded"])
    ]
    subsets = dict(
        all=all_pairs, selected=[p for p in all_pairs if p["id"] in selected]
    )
    report = dict(samples=samples, seed=(settings["seed"] + 100003) % 2**32, results={})
    completed = 0
    for subset, pairs in subsets.items():
        output = dict(
            link_pairs=len(pairs), object_pairs=sum(len(p["objects"]) for p in pairs)
        )
        for mode in ("collision", "distance"):
            for pose in poses[: min(10, samples)]:
                robot.place(pose)
                for p in pairs:
                    robot.evaluate(p, mode)
            timings = []
            for pose in poses:
                if cancelled():
                    return None
                start = time.perf_counter_ns()
                robot.place(pose)
                for p in pairs:
                    robot.evaluate(p, mode)
                timings.append((time.perf_counter_ns() - start) / 1e6)
                completed += 1
                if completed % 10 == 0:
                    progress(completed / (4 * samples), "测速中")
            output[mode] = dict(
                mean_ms=float(np.mean(timings)),
                p95_ms=float(np.percentile(timings, 95)),
            )
        report["results"][subset] = output
    report["speedup"] = {
        mode: report["results"]["all"][mode]["mean_ms"]
        / max(report["results"]["selected"][mode]["mean_ms"], 1e-12)
        for mode in ("collision", "distance")
    }
    return report


def worker(payload, queue, cancel_event):
    try:
        robot = Robot(**payload["source"])
        if robot.fingerprint != payload["fingerprint"]:
            raise ValueError("模型资源已变化，请重新加载")
        progress = lambda fraction, message: queue.put(
            dict(type="progress", progress=fraction, message=message)
        )
        if payload["kind"] == "analysis":
            result = analyze(
                robot,
                payload["settings"],
                payload["overrides"],
                progress,
                cancel_event.is_set,
            )
        else:
            result = benchmark(
                robot,
                payload["settings"],
                payload["selected"],
                payload["samples"],
                progress,
                cancel_event.is_set,
            )
        queue.put(
            dict(type="cancelled" if result is None else "complete", result=result)
        )
    except Exception as exc:
        queue.put(dict(type="error", message=f"{type(exc).__name__}: {exc}"))
