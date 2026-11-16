"""Local-only API, background process lifecycle, and production UI serving."""

from __future__ import annotations

import asyncio
import copy
import json
import multiprocessing as mp
import queue
import threading
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from .analysis import validate_settings, worker
from .export import InfoDistanceConfirmation, atomic_text, export_files
from .model import Robot
from .state import Session

ROOT = Path(__file__).resolve().parent.parent


class Store:
    def __init__(self):
        self.lock = threading.RLock()
        self.session = None
        self.job = None
        self.process = self.cancel_event = None

    def require(self, body=None):
        if self.session is None:
            raise ValueError("请先加载 URDF")
        if (
            body
            and body.get("revision", self.session.revision) != self.session.revision
        ):
            raise ValueError("模型已切换，请刷新")
        return self.session

    def busy(self):
        return self.job is not None and self.job["status"] in ("running", "cancelling")

    def stop(self):
        if self.cancel_event:
            self.cancel_event.set()
        if self.process:
            self.process.join(timeout=1)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1)

    def start(self, body):
        session = self.require(body)
        if self.busy():
            raise ValueError("已有分析或测速任务，请先取消或等待完成")
        validate_settings(session.robot, session.settings)
        kind = body.get("kind", "analysis")
        if kind not in ("analysis", "benchmark"):
            raise ValueError("未知任务类型")
        samples = body.get("samples", 200)
        if not isinstance(samples, int) or not 1 <= samples <= 10000:
            raise ValueError("测速样本数必须在 1 到 10000 之间")
        revision, generation = session.revision, session.generation
        payload = dict(
            source=dict(
                path=session.robot.path,
                packages=session.robot.packages,
                xacro_args=session.robot.xacro_args,
            ),
            fingerprint=session.robot.fingerprint,
            settings=copy.deepcopy(session.settings),
            kind=kind,
            samples=samples,
            overrides={
                key: c["min_distance"]
                for key, c in session.choices.items()
                if c["min_distance"] is not None
            },
            selected=session.selected(),
        )
        ctx = mp.get_context("spawn")
        events, cancel = ctx.Queue(), ctx.Event()
        process = ctx.Process(
            target=worker, args=(payload, events, cancel), daemon=True
        )
        job = dict(
            id=uuid.uuid4().hex,
            kind=kind,
            status="running",
            progress=0.0,
            message="正在准备模型",
            started=time.time(),
        )
        self.process, self.cancel_event, self.job = process, cancel, job
        process.start()

        def monitor():
            try:
                while True:
                    try:
                        event = events.get(timeout=0.2)
                    except queue.Empty:
                        if process.is_alive():
                            continue
                        # A crashed native worker must not leave a permanently running task.
                        with self.lock:
                            job.update(
                                status="cancelled" if cancel.is_set() else "error",
                                message="任务已取消"
                                if cancel.is_set()
                                else f"计算进程退出 ({process.exitcode})",
                            )
                        break
                    with self.lock:
                        if event["type"] == "progress":
                            job.update(
                                progress=event["progress"], message=event["message"]
                            )
                            continue
                        status = event["type"]
                        current = self.session
                        if cancel.is_set():
                            status = "cancelled"
                        elif status == "complete":
                            if (
                                current is None
                                or current.revision != revision
                                or current.generation != generation
                            ):
                                status = "obsolete"
                            elif kind == "analysis":
                                current.accept_analysis(event["result"])
                            else:
                                current.report = event["result"]
                        job.update(
                            status=status,
                            progress=1.0 if status == "complete" else job["progress"],
                            message=event.get(
                                "message",
                                {
                                    "complete": "完成",
                                    "obsolete": "设置已变化，结果未应用",
                                    "cancelled": "任务已取消",
                                }.get(status, status),
                            ),
                        )
                        break
            finally:
                process.join(timeout=2)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=1)
                events.close()

        threading.Thread(target=monitor, daemon=True).start()
        return job


store = Store()


@asynccontextmanager
async def lifespan(app):
    yield
    store.stop()


app = FastAPI(title="URDF Self Collision Checker", lifespan=lifespan)
app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=["127.0.0.1", "localhost", "[::1]", "testserver"],
)


@app.middleware("http")
async def local_mutations(request: Request, call_next):
    if request.method in {"POST", "PATCH", "DELETE", "PUT"}:
        # Browser cross-origin writes to a localhost file API must be rejected.
        origin = request.headers.get("origin")
        if origin:
            from urllib.parse import urlparse

            if urlparse(origin).hostname not in {"127.0.0.1", "localhost", "::1"}:
                return JSONResponse({"detail": "仅接受本地界面请求"}, status_code=403)
        if "application/json" not in request.headers.get("content-type", ""):
            return JSONResponse(
                {"detail": "请求必须使用 application/json"}, status_code=415
            )
    return await call_next(request)


@app.exception_handler(ValueError)
async def value_error(request, exc):
    return JSONResponse({"detail": str(exc)}, status_code=400)


@app.exception_handler(OSError)
async def os_error(request, exc):
    return JSONResponse({"detail": str(exc)}, status_code=400)


@app.exception_handler(KeyError)
async def key_error(request, exc):
    return JSONResponse({"detail": f"缺少字段或未知标识：{exc}"}, status_code=400)


@app.exception_handler(RuntimeError)
async def native_error(request, exc):
    return JSONResponse({"detail": f"模型加载或计算失败：{exc}"}, status_code=400)


@app.get("/api/state")
def state():
    with store.lock:
        return dict(
            session=None if store.session is None else store.session.public(),
            job=store.job,
        )


@app.get("/api/browse")
def browse(path: str = "."):
    directory = Path(path).expanduser().resolve()
    entries = sorted(
        directory.iterdir(), key=lambda p: (not p.is_dir(), p.name.lower())
    )
    return dict(
        path=str(directory),
        parent=str(directory.parent),
        entries=[
            dict(name=p.name, path=str(p), directory=p.is_dir())
            for p in entries
            if not p.name.startswith(".")
            and (p.is_dir() or p.suffix in {".urdf", ".xacro", ".json"})
        ],
    )


@app.post("/api/load")
def load(body: dict):
    robot = Robot(body["path"], body.get("packages"), body.get("xacro_args"))
    with store.lock:
        store.stop()
        store.session = Session(robot, uuid.uuid4().hex)
        store.job = None
        return store.session.public()


@app.get("/api/assets/{revision}/{asset_id}/{filename}")
def asset(revision: str, asset_id: str, filename: str):
    with store.lock:
        session = store.require({"revision": revision})
        if asset_id not in session.robot.assets:
            raise ValueError("未知模型资源")
        if Path(session.robot.assets[asset_id]).suffix.lower() == ".dae":
            return Response(
                session.robot.browser_collada(asset_id, revision),
                media_type="model/vnd.collada+xml",
            )
        return FileResponse(session.robot.assets[asset_id])


@app.patch("/api/settings")
def settings(body: dict):
    with store.lock:
        s = store.require(body)
        s.update_settings(body["settings"])
        return s.public()


@app.patch("/api/pairs")
def pairs(body: dict):
    with store.lock:
        s = store.require(body)
        # Validate on a copy for all-or-nothing bulk mutations.
        choices = copy.deepcopy(s.choices)
        saved = (s.stale, s.generation, s.undo, s.report)
        try:
            for key in body["ids"]:
                s.update_pair(key, dict(body["patch"]))
        except Exception:
            s.choices = choices
            s.stale, s.generation, s.undo, s.report = saved
            raise
        return s.public()


@app.post("/api/reset-selection")
def reset_selection(body: dict):
    with store.lock:
        s = store.require(body)
        for c in s.choices.values():
            c.update(manual=None, lock=None, topk=False)
        s.generation += 1
        s.undo, s.report = None, None
        return s.public()


@app.post("/api/pose")
def pose(body: dict):
    with store.lock:
        s = store.require(body)
        s.robot.place(body["pose"])
        key = body.get("pair_id")
        result = s.robot.evaluate(s.robot.pairs[key]) if key in s.robot.pairs else None
        frames = {
            name: s.robot.data.oMf[s.robot.model.getFrameId(name)].homogeneous.tolist()
            for name in s.robot.links
        }
        geometry_frames = {
            str(i): matrix.homogeneous.tolist()
            for i, matrix in enumerate(s.robot.geom_data.oMg)
        }
        return dict(
            pose=s.robot.pose,
            result=result,
            frames=frames,
            geometry_frames=geometry_frames,
        )


@app.post("/api/jobs")
def start_job(body: dict):
    with store.lock:
        return store.start(body)


@app.post("/api/jobs/cancel")
def cancel_job(body: dict):
    with store.lock:
        if store.busy():
            store.cancel_event.set()
            store.job.update(status="cancelling", message="正在取消")
        return store.job


@app.post("/api/topk")
def topk(body: dict):
    with store.lock:
        return store.require(body).topk(body["k"], body.get("apply", False))


@app.post("/api/undo")
def undo(body: dict):
    with store.lock:
        s = store.require(body)
        if s.undo is None:
            raise ValueError("没有可以撤销的筛选")
        s.choices, s.undo = s.undo, None
        s.generation += 1
        s.report = None
        return s.public()


@app.post("/api/export")
def export(body: dict):
    with store.lock:
        try:
            return export_files(
                store.require(body),
                body.get("directory", str(ROOT / "config")),
                body.get("acknowledge_info_global_distance", False),
            )
        except InfoDistanceConfirmation as exc:
            return JSONResponse(
                dict(detail=str(exc), code="INFO_GLOBAL_DISTANCE_CONFIRMATION"),
                status_code=409,
            )


@app.post("/api/project/save")
def save_project(body: dict):
    with store.lock:
        path = Path(body["path"]).expanduser().resolve()
        atomic_text(
            path,
            json.dumps(
                store.require(body).project(),
                ensure_ascii=False,
                allow_nan=False,
                indent=2,
            ),
        )
        return dict(path=str(path))


@app.post("/api/project/open")
def open_project(body: dict):
    project = json.loads(Path(body["path"]).expanduser().read_text())
    if project.get("version") != 1:
        raise ValueError("不支持的工程版本")
    robot = Robot(**project["source"])
    s = Session(robot, uuid.uuid4().hex)
    warning = None
    if (
        robot.fingerprint != project["fingerprint"]
        or robot.public(s.revision)["versions"] != project["versions"]
    ):
        warning = (
            "模型资源或计算库版本已变化；旧采样、选择及 object 索引已失效，请重新分析"
        )
    else:
        if set(project["choices"]) != set(s.choices):
            raise ValueError("工程碰撞对与模型不一致")
        s.update_settings(project["settings"])
        for key, choice in project["choices"].items():
            s.update_pair(key, choice)
        s.stats, s.stale = project["stats"], project["stale"]
        s.elapsed_seconds, s.report = (
            project["elapsed_seconds"],
            project.get("benchmark"),
        )
        robot.place(project.get("pose", robot.default_pose))
    with store.lock:
        store.stop()
        store.session, store.job = s, None
        return dict(session=s.public(), warning=warning)


@app.websocket("/ws")
async def websocket(ws: WebSocket):
    origin = ws.headers.get("origin", "")
    if origin:
        from urllib.parse import urlparse

        if urlparse(origin).hostname not in {"localhost", "127.0.0.1", "::1"}:
            await ws.close(code=1008)
            return
    await ws.accept()
    previous = None
    try:
        while True:
            with store.lock:
                payload = json.dumps(store.job, ensure_ascii=False)
            if payload != previous:
                await ws.send_text(payload)
                previous = payload
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass


if (ROOT / "frontend" / "dist").is_dir():
    app.mount(
        "/", StaticFiles(directory=ROOT / "frontend" / "dist", html=True), name="ui"
    )
else:

    @app.get("/")
    def missing_ui():
        return {
            "message": "请先运行 npm --prefix frontend install && npm --prefix frontend run build；开发时访问 Vite 地址。"
        }
