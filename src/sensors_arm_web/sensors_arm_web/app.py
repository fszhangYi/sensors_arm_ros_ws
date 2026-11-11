"""FastAPI app: same UI PAGE as sensors-arm, ROS-backed APIs."""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path
from typing import Any, List, Optional

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from sensors_arm_web.backend import RosOrchestrator


def _load_page(sensors_arm_root: str) -> str:
    root = Path(sensors_arm_root)
    src = root / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))
    from sensors_arm.viz import PAGE  # type: ignore
    return PAGE


class ArmSessionBody(BaseModel):
    arm_id: str


class ArmCommandBody(BaseModel):
    arm_id: str
    arm: Optional[bool] = None
    disarm: Optional[bool] = None
    stop: Optional[bool] = None
    joints_rad: Optional[List[float]] = None
    jog_joint: Optional[int] = None
    delta_deg: Optional[float] = None
    delta_rad: Optional[float] = None
    cancel_abs_ramp: bool = False
    timing: Optional[str] = None
    duration_s: Optional[float] = None
    t_min_s: Optional[float] = None
    t_max_s: Optional[float] = None
    v_norm_rad_s: Optional[float] = None
    jerk_seg_frac: Optional[float] = None
    accel_seg_frac: Optional[float] = None


class ArmHomeBody(BaseModel):
    arm_id: str
    joints_rad: Optional[List[float]] = None
    t_min_s: Optional[float] = None
    t_max_s: Optional[float] = None
    v_norm_rad_s: Optional[float] = None
    jerk_seg_frac: Optional[float] = None
    accel_seg_frac: Optional[float] = None


class ArmIkBody(BaseModel):
    arm_id: str
    xyzrpy: List[float]
    seed_joints_rad: Optional[List[float]] = None
    ori_weight: Optional[float] = None
    orientation_tolerance_rad: Optional[float] = None
    # sensors-arm UI sends a float residual threshold (e.g. 8e-3), not bool.
    accept_residual: Optional[float] = None


class ArmMaxDeltaBody(BaseModel):
    arm_id: str
    max_delta_deg: float


class ArmTeleopStreamBody(BaseModel):
    arm_id: str
    enabled: bool = False
    joints_rad: Optional[List[float]] = None
    hz: float = 50.0


class ArmTeleopSetpointBody(BaseModel):
    arm_id: str
    joints_rad: List[float]


class FovCameraBody(BaseModel):
    arm_id: str
    camera: dict = Field(default_factory=dict)


class FovAnalyzeBody(BaseModel):
    arm_id: str
    camera: Optional[dict] = None


class FovSyncStlBody(BaseModel):
    arm_id: str
    write_yaml: bool = True


def create_app(orch: RosOrchestrator, *, sensors_arm_root: str) -> FastAPI:
    page = _load_page(sensors_arm_root)
    app = FastAPI(title="sensors-arm-ros2", docs_url="/api/docs")
    assets = Path(sensors_arm_root) / "src" / "sensors_arm" / "static"
    if assets.is_dir():
        app.mount("/assets", StaticFiles(directory=str(assets), follow_symlink=True), name="assets")

    @app.get("/", response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse(page)

    @app.get("/api/health")
    async def health() -> dict:
        return {"ok": True, "app": "sensors-arm-ros2"}

    @app.get("/api/status")
    async def status() -> dict:
        return orch.status()

    @app.get("/api/arms")
    async def arms() -> dict:
        return {"arms": orch.arms_descriptor()}

    @app.post("/api/arm/connect")
    async def arm_connect(body: ArmSessionBody) -> dict:
        return await asyncio.to_thread(orch.arm_connect, body.arm_id)

    @app.post("/api/arm/disconnect")
    async def arm_disconnect(body: ArmSessionBody) -> dict:
        return await asyncio.to_thread(orch.arm_disconnect, body.arm_id)

    @app.post("/api/arm/command")
    async def arm_cmd(body: ArmCommandBody) -> dict:
        return await asyncio.to_thread(
            orch.arm_command,
            body.arm_id,
            arm=body.arm,
            disarm=body.disarm,
            stop=body.stop,
            joints_rad=body.joints_rad,
            jog_joint=body.jog_joint,
            delta_deg=body.delta_deg,
            delta_rad=body.delta_rad,
            cancel_abs_ramp=body.cancel_abs_ramp,
            timing=body.timing,
            duration_s=body.duration_s,
        )

    @app.get("/api/arm/home")
    async def arm_home(arm_id: str) -> dict:
        return orch.arm_home_get(arm_id)

    @app.post("/api/arm/home/set")
    async def arm_home_set(body: ArmHomeBody) -> dict:
        return orch.arm_home_set(body.arm_id, body.joints_rad)

    @app.post("/api/arm/home/go")
    async def arm_home_go(body: ArmHomeBody) -> dict:
        return await asyncio.to_thread(orch.arm_home_go, body.arm_id)

    @app.get("/api/arm/abs-ramp")
    async def arm_abs_ramp_get() -> dict:
        return orch.status().get("arm_abs_ramp") or {}

    @app.post("/api/arm/ik")
    async def arm_ik(body: ArmIkBody) -> dict:
        return await asyncio.to_thread(
            orch.arm_ik,
            body.arm_id,
            body.xyzrpy,
            body.seed_joints_rad,
            ori_weight=body.ori_weight,
            orientation_tolerance_rad=body.orientation_tolerance_rad,
            accept_residual=body.accept_residual,
        )

    @app.get("/api/arm/max-delta")
    async def arm_max_delta_get(arm_id: str) -> dict:
        return orch.arm_max_delta_get(arm_id)

    @app.post("/api/arm/max-delta")
    async def arm_max_delta_set(body: ArmMaxDeltaBody) -> dict:
        return orch.arm_max_delta_set(body.arm_id, body.max_delta_deg)

    @app.get("/api/arm/teleop-stream")
    async def arm_teleop_stream_get(arm_id: Optional[str] = None) -> dict:
        return orch.teleop_stream_status(arm_id)

    @app.post("/api/arm/teleop-stream")
    async def arm_teleop_stream_set(body: ArmTeleopStreamBody) -> dict:
        return await asyncio.to_thread(
            orch.teleop_stream_set,
            body.arm_id,
            enabled=body.enabled,
            joints_rad=body.joints_rad,
            hz=body.hz,
        )

    @app.post("/api/arm/teleop-stream/setpoint")
    async def arm_teleop_stream_setpoint(body: ArmTeleopSetpointBody) -> dict:
        return await asyncio.to_thread(orch.teleop_stream_setpoint, body.arm_id, body.joints_rad)

    @app.get("/api/fov/camera")
    async def fov_camera_get(arm_id: str) -> Any:
        try:
            cam = orch.arm_camera_get(arm_id)
        except KeyError as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=404)
        return {"ok": True, "arm_id": arm_id, "camera": cam}

    @app.post("/api/fov/camera")
    async def fov_camera_set(body: FovCameraBody) -> dict:
        return orch.arm_camera_set(body.arm_id, body.camera)

    @app.post("/api/fov/analyze")
    async def fov_analyze_api(body: FovAnalyzeBody) -> dict:
        return await asyncio.to_thread(orch.arm_fov_analyze, body.arm_id, body.camera)

    @app.post("/api/fov/sync-stl")
    async def fov_sync_stl_api(body: FovSyncStlBody) -> dict:
        return await asyncio.to_thread(orch.arm_fov_sync_stl, body.arm_id, write_yaml=body.write_yaml)

    @app.post("/api/fov/export")
    async def fov_export_api(body: FovAnalyzeBody) -> Response:
        out = await asyncio.to_thread(orch.arm_fov_export, body.arm_id, body.camera)
        if not out.get("ok"):
            return JSONResponse(
                {"ok": False, "arm_id": body.arm_id, "error": out.get("error") or "export failed"},
                status_code=400,
            )
        data = out["zip_bytes"]
        fname = str(out.get("filename") or "gripper_open_close.mp4")
        media = str(out.get("media_type") or "video/mp4")
        has_vid = "1" if out.get("has_video") else "0"
        warn = str(out.get("warning") or "")
        return Response(
            content=data,
            media_type=media,
            headers={
                "Content-Disposition": 'attachment; filename="{}"'.format(fname),
                "Content-Length": str(len(data)),
                "X-Fov-Has-Video": has_vid,
                "X-Fov-Warning": warn,
                "Access-Control-Expose-Headers": "Content-Disposition, X-Fov-Has-Video, X-Fov-Warning",
            },
        )

    @app.post("/api/shutdown")
    async def shutdown() -> dict:
        return orch.request_shutdown()

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        last_seq = -1
        try:
            while True:
                snap = orch.hub.snapshot()
                if snap is not None:
                    seq = int(snap.get("_hub_seq") or 0)
                    if seq != last_seq:
                        last_seq = seq
                        await ws.send_json(snap)
                await asyncio.sleep(1.0 / 15.0)
        except WebSocketDisconnect:
            return
        except Exception:
            return

    @app.exception_handler(KeyError)
    async def _key(request: Request, exc: KeyError) -> JSONResponse:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=404)

    return app
