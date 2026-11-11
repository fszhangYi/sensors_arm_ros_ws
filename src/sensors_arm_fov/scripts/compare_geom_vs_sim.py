#!/usr/bin/env python3
"""Offline regression: geometric az/el box vs CameraInfo projection (not product path)."""

from __future__ import annotations

import argparse
import math
import sys

import yaml

# Allow running from source tree without install
sys.path.insert(0, __file__.rsplit("/scripts", 1)[0])

from sensors_arm_fov.projection import (  # noqa: E402
    analyze_projection,
    axes_from_cfg,
    in_image,
    project_angular,
    project_pixel,
    resolve_mode_optics,
    tool_to_optical_matrix,
)
import numpy as np


def geom_in_fov(d, az, el, H, V, min_depth=1.0):
    return d > min_depth and abs(az) <= H / 2.0 and abs(el) <= V / 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", required=True)
    args = ap.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    lens, look, right, up = axes_from_cfg(cfg)
    R = tool_to_optical_matrix(look, right, up)
    proj = analyze_projection(cfg)
    print("mode\tpoint\tgeom\tsim\tmatch")
    mismatches = 0
    for m in cfg["modes"]:
        opt = resolve_mode_optics(m)
        for s in cfg["samples"]:
            xyz = np.asarray(s["xyz"], float)
            d, az, el = project_angular(xyz, lens, look, right, up)
            g = geom_in_fov(d, az, el, opt["H_deg"], opt["V_deg"])
            p_opt = (R @ (xyz - lens)) / 1000.0
            u, v, z = project_pixel(p_opt, opt["fx"], opt["fy"], opt["width"] / 2, opt["height"] / 2)
            s_ok = in_image(u, v, z, opt["width"], opt["height"])
            match = g == s_ok
            if not match:
                mismatches += 1
            print("{}\t{}\t{}\t{}\t{}".format(m["id"], s["name"], g, s_ok, match))
    print("recommend(sim)=", proj.get("recommend_mode"), "mismatches=", mismatches)
    return 0 if mismatches == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
