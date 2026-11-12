#!/usr/bin/env python3
"""Entry point: serve sensors-arm UI backed by ROS2."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import uvicorn
import yaml


def _default_sensors_arm_root() -> str:
    """Prefer SENSORS_ARM_ROOT / <ws>/sensors-arm symlink (scripts/link_deps.sh)."""
    env = (os.environ.get("SENSORS_ARM_ROOT") or "").strip()
    if env and os.path.isdir(env):
        return env
    ws = (os.environ.get("SENSORS_ARM_ROS_WS") or "").strip()
    if ws:
        cand = os.path.join(ws, "sensors-arm")
        if os.path.isdir(cand):
            return cand
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "sensors-arm"
        if (cand / "src" / "sensors_arm").is_dir() or (cand / "configs").is_dir():
            return str(cand)
    return str(Path.home() / "autodl-tmp" / "sensors-arm")


def _resolve_fov_yaml(explicit: str) -> str:
    if explicit and os.path.isfile(explicit):
        return explicit
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("sensors_arm_fov")
        cand = os.path.join(share, "config", "cell_c_fov.yaml")
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    # Source-tree fallback
    ws = Path(__file__).resolve().parents[2]
    cand = ws / "sensors_arm_fov" / "config" / "cell_c_fov.yaml"
    return str(cand)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="sensors_arm_web server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=6008)
    parser.add_argument("--config", default="", help="web.yaml")
    parser.add_argument("--rosbridge", default="ws://127.0.0.1:9090")
    parser.add_argument("--mode", default="sim", choices=["sim", "hardware", "dry_run"])
    args = parser.parse_args(argv)

    cfg = {}
    if args.config and os.path.isfile(args.config):
        with open(args.config) as f:
            cfg = yaml.safe_load(f) or {}
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("sensors_arm_web")
            default_cfg = os.path.join(share, "config", "web.yaml")
            if os.path.isfile(default_cfg):
                with open(default_cfg) as f:
                    cfg = yaml.safe_load(f) or {}
        except Exception:
            src_cfg = Path(__file__).resolve().parents[1] / "config" / "web.yaml"
            if src_cfg.is_file():
                with open(src_cfg) as f:
                    cfg = yaml.safe_load(f) or {}

    host = args.host or cfg.get("host") or "0.0.0.0"
    port = int(args.port or cfg.get("port") or 6008)
    rosbridge = args.rosbridge or cfg.get("rosbridge_url") or "ws://127.0.0.1:9090"
    mode = args.mode or cfg.get("mode") or "sim"
    sensors_arm_root = (cfg.get("sensors_arm_root") or "").strip() or _default_sensors_arm_root()
    default_yaml = (cfg.get("default_yaml") or "").strip() or os.path.join(
        sensors_arm_root, "configs/default.yaml"
    )
    models_catalog = (cfg.get("models_catalog") or "").strip() or os.path.join(
        sensors_arm_root, "configs/models_catalog.yaml"
    )
    fov_yaml = _resolve_fov_yaml(cfg.get("fov_yaml") or "")

    # Prefer workspace venv uvicorn/fastapi when available
    from sensors_arm_web.app import create_app
    from sensors_arm_web.backend import RosOrchestrator

    orch = RosOrchestrator(
        default_yaml=default_yaml,
        models_catalog=models_catalog,
        fov_yaml=fov_yaml,
        rosbridge_url=rosbridge,
        mode=mode,
    )
    orch.start()
    app = create_app(orch, sensors_arm_root=sensors_arm_root)
    print(
        "[sensors_arm_web] http://{}:{} rosbridge={} sensors-arm={} fov={}".format(
            host, port, rosbridge, sensors_arm_root, fov_yaml
        ),
        flush=True,
    )
    try:
        uvicorn.run(app, host=host, port=port, log_level="info")
    finally:
        orch.stop()


if __name__ == "__main__":
    main()
