#!/usr/bin/env python3
"""CLI: call fov/validate or run offline projection from YAML."""

from __future__ import annotations

import argparse
import json
import sys


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="sensors_arm_fov CLI")
    parser.add_argument("-c", "--config", required=True, help="cell_c_fov.yaml")
    parser.add_argument("--mode", default="", help="mode id")
    parser.add_argument("--zip", default="", help="write export zip path")
    parser.add_argument("--ros", action="store_true", help="call ROS service fov/validate")
    args = parser.parse_args(argv)

    if args.ros:
        import rclpy
        from sensors_arm_msgs.srv import FovValidate

        rclpy.init()
        node = rclpy.create_node("fov_cli")
        client = node.create_client(FovValidate, "fov/validate")
        if not client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error("fov/validate unavailable")
            return 1
        req = FovValidate.Request()
        req.arm_id = "arm-c"
        req.mode_id = args.mode
        req.export_zip = bool(args.zip)
        req.use_image_occlusion = False
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None or not res.ok:
            print((res.error if res else "no response"), file=sys.stderr)
            return 2
        print(res.result_json)
        if args.zip and res.zip_bytes:
            with open(args.zip, "wb") as f:
                f.write(bytes(res.zip_bytes))
            print("wrote", args.zip, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    import yaml
    from sensors_arm_fov.projection import analyze_projection, build_export_zip

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)
    ov = None
    if args.mode:
        for m in cfg.get("modes") or []:
            if m.get("id") == args.mode:
                ov = dict(m)
                break
    result = analyze_projection(cfg, mode_overrides=ov)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.zip:
        data = build_export_zip(result, cfg, arm_id=cfg.get("arm_id", "arm-c"))
        with open(args.zip, "wb") as f:
            f.write(data)
        print("wrote", args.zip, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
