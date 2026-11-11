#!/usr/bin/env python3
"""Render a clean layered architecture PNG/SVG for sensors_arm_ros_ws."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
from matplotlib import font_manager

OUT = Path(__file__).resolve().parent


def _pick_font() -> str:
    for name in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "Noto Sans CJK JP"):
        path = font_manager.findfont(name)
        if path and "DejaVu" not in path:
            return name
    return "DejaVu Sans"


FONT = _pick_font()
plt.rcParams["font.family"] = FONT
plt.rcParams["axes.unicode_minus"] = False


def band(ax, y, h, color, title):
    ax.add_patch(
        Rectangle((0.03, y), 0.94, h, facecolor=color, edgecolor="#00000018", lw=0.8, zorder=1)
    )
    ax.text(
        0.045,
        y + h - 0.012,
        title,
        ha="left",
        va="top",
        fontsize=10,
        fontweight="bold",
        color="#1c2833",
        zorder=2,
    )


def box(ax, x, y, w, h, text, *, fc="#fff", ec="#333", lw=1.2, fs=9, dashed=False):
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle="round,pad=0.01,rounding_size=0.018",
            facecolor=fc,
            edgecolor=ec,
            linewidth=lw,
            linestyle="--" if dashed else "-",
            zorder=3,
        )
    )
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, color="#1c2833", zorder=4, linespacing=1.3)
    return dict(cx=x + w / 2, top=y + h, bot=y, left=x, right=x + w, mid_y=y + h / 2)


def arrow(ax, x1, y1, x2, y2, *, color="#5d6d7e", style="-", lw=1.5, label=None, label_xy=None):
    ax.annotate(
        "",
        xy=(x2, y2),
        xytext=(x1, y1),
        arrowprops=dict(
            arrowstyle="-|>",
            color=color,
            lw=lw,
            linestyle=style,
            shrinkA=1,
            shrinkB=1,
            mutation_scale=11,
        ),
        zorder=5,
    )
    if label:
        lx, ly = label_xy if label_xy else ((x1 + x2) / 2, (y1 + y2) / 2)
        ax.text(lx, ly, label, ha="center", va="bottom", fontsize=7.5, color=color, zorder=6)


def main() -> None:
    fig, ax = plt.subplots(figsize=(11.5, 13.2), dpi=170)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    fig.patch.set_facecolor("#f7f9fb")

    ax.text(0.5, 0.985, "sensors_arm_ros_ws 分层架构", ha="center", va="top", fontsize=16, fontweight="bold", color="#1b2631")
    ax.text(
        0.5,
        0.960,
        "sensors-arm = UI / 资产来源　　本仓 = ROS 2 Foxy + Gazebo 接管控制与 FOV",
        ha="center",
        va="top",
        fontsize=9,
        color="#566573",
    )

    band(ax, 0.875, 0.070, "#d6eaf8", "① 使用层")
    band(ax, 0.755, 0.100, "#d0ece7", "② Web 层（复用 sensors-arm）")
    band(ax, 0.655, 0.080, "#fcf3cf", "③ 桥接层")
    band(ax, 0.430, 0.205, "#d5f5e3", "④ ROS 应用层（本仓库 packages）")
    band(ax, 0.265, 0.140, "#fdebd0", "⑤ 执行层")
    band(ax, 0.105, 0.135, "#e8daef", "⑥ 共享依赖（仓外）")

    # L1
    b_browser = box(ax, 0.35, 0.890, 0.30, 0.040, "浏览器  :6008", fc="#aed6f1", ec="#1a5276", fs=10)

    # L2
    b_web = box(
        ax, 0.16, 0.775, 0.40, 0.055, "sensors_arm_web\nFastAPI · 同款 Three.js UI", fc="#a3e4d7", ec="#0e6655", fs=9
    )
    b_assets = box(
        ax,
        0.60,
        0.775,
        0.30,
        0.055,
        "sensors-arm 资产\nPAGE · /assets · YAML",
        fc="#d6eaf8",
        ec="#1a5276",
        fs=8.5,
        dashed=True,
    )

    # L3
    b_rb = box(
        ax,
        0.26,
        0.670,
        0.48,
        0.045,
        "rosbridge_websocket  :9090   ·   JSON ↔ ROS",
        fc="#f9e79f",
        ec="#9a7d0a",
        fs=9,
    )

    # L4 packages — two rows to avoid crowding
    # Row A: runtime services
    # Row B: model / sim support + bringup/msgs
    y_a, y_b = 0.545, 0.455
    w, h = 0.165, 0.055
    gap = 0.02
    xs = [0.06, 0.06 + (w + gap), 0.06 + 2 * (w + gap), 0.06 + 3 * (w + gap), 0.06 + 4 * (w + gap)]

    b_control = box(ax, xs[0], y_a, w, h, "control\n关节 · ramp · IK", fc="#eafaf1", ec="#1e8449", fs=8)
    b_fov = box(ax, xs[1], y_a, w, h, "fov\nCameraInfo 投影", fc="#eafaf1", ec="#1e8449", fs=8)
    b_hw = box(ax, xs[2], y_a, w, h, "hw\nhik-sensors 桥", fc="#eafaf1", ec="#1e8449", fs=8)
    b_desc = box(ax, xs[3], y_a, w, h, "description\nURDF / mesh", fc="#eafaf1", ec="#1e8449", fs=8)
    b_gzpkg = box(ax, xs[4], y_a, w, h, "gazebo\nworld · D405", fc="#eafaf1", ec="#1e8449", fs=8)

    b_bringup = box(ax, 0.22, y_b, 0.24, 0.045, "bringup · 一键 launch 组装", fc="#82e0aa", ec="#1e8449", fs=8.5)
    b_msgs = box(ax, 0.54, y_b, 0.24, 0.045, "msgs · Arm* / Fov* 接口", fc="#eafaf1", ec="#1e8449", fs=8.5)
    ax.text(0.50, y_b + 0.055, "（点线：bringup 拉起上层各节点；msgs 为共享契约）", ha="center", va="bottom", fontsize=7, color="#1e8449")

    # L5
    b_gz = box(
        ax,
        0.10,
        0.285,
        0.42,
        0.080,
        "Gazebo Classic 11\nCell C · D405 · /cell_c/d405/*",
        fc="#f5cba7",
        ec="#a04000",
        fs=9,
    )
    b_robot = box(
        ax,
        0.58,
        0.285,
        0.32,
        0.080,
        "真机 / dry-run\nElite CS66 / EC616",
        fc="#f5b7b1",
        ec="#922b21",
        fs=9,
    )

    # L6
    b_sa = box(
        ax,
        0.10,
        0.125,
        0.42,
        0.080,
        "sensors-arm（产品仓）\n原非 ROS · 提供 UI / 配置 / 网格",
        fc="#d6eaf8",
        ec="#1a5276",
        fs=8.5,
        dashed=True,
    )
    b_sensors = box(
        ax,
        0.58,
        0.125,
        0.32,
        0.080,
        "hik-sensors\n驱动唯一归属 · SENSORS_SRC",
        fc="#d7bde2",
        ec="#6c3483",
        fs=8.5,
    )

    # ===== Main path arrows (vertical, few) =====
    arrow(ax, b_browser["cx"], b_browser["bot"], b_web["cx"], b_web["top"], color="#2471a3", lw=2.0)
    arrow(
        ax,
        b_assets["left"],
        b_assets["mid_y"],
        b_web["right"],
        b_web["mid_y"],
        color="#1a5276",
        style="--",
        lw=1.6,
        label="复用",
        label_xy=(0.56, 0.805),
    )
    arrow(ax, b_web["cx"], b_web["bot"], b_rb["cx"], b_rb["top"], color="#148f77", lw=2.0)

    # rosbridge fans to control/fov/hw only
    arrow(ax, b_rb["cx"] - 0.08, b_rb["bot"], b_control["cx"], b_control["top"], color="#b7950b", lw=1.4)
    arrow(ax, b_rb["cx"], b_rb["bot"], b_fov["cx"], b_fov["top"], color="#b7950b", lw=1.4)
    arrow(ax, b_rb["cx"] + 0.08, b_rb["bot"], b_hw["cx"], b_hw["top"], color="#b7950b", lw=1.4)

    # down to execution
    arrow(ax, b_control["cx"], b_control["bot"], 0.22, b_gz["top"], color="#a04000", lw=1.35, label="sim", label_xy=(0.16, 0.42))
    arrow(ax, b_fov["cx"], b_fov["bot"], 0.32, b_gz["top"], color="#a04000", lw=1.35, label="CameraInfo", label_xy=(0.34, 0.42))
    arrow(ax, b_desc["cx"], b_desc["bot"], 0.42, b_gz["top"], color="#a04000", lw=1.2, label="URDF", label_xy=(0.48, 0.42))
    arrow(ax, b_gzpkg["cx"], b_gzpkg["bot"], 0.48, b_gz["top"], color="#a04000", lw=1.2)
    arrow(ax, b_hw["cx"], b_hw["bot"], b_robot["cx"], b_robot["top"], color="#922b21", lw=1.5)
    arrow(ax, b_robot["cx"], b_robot["bot"], b_sensors["cx"], b_sensors["top"], color="#6c3483", lw=1.5)

    # sensors-arm → description meshes：只走图右侧边距，避免穿过中间层
    gutter = 0.975
    ax.plot(
        [b_sa["right"], gutter, gutter, b_desc["right"]],
        [b_sa["mid_y"], b_sa["mid_y"], b_desc["mid_y"], b_desc["mid_y"]],
        color="#1a5276",
        linestyle="--",
        lw=1.25,
        zorder=5,
        clip_on=False,
    )
    ax.annotate(
        "",
        xy=(b_desc["right"], b_desc["mid_y"]),
        xytext=(b_desc["right"] + 0.001, b_desc["mid_y"]),
        arrowprops=dict(arrowstyle="-|>", color="#1a5276", lw=1.25, mutation_scale=9),
        zorder=5,
        clip_on=False,
    )
    ax.text(
        gutter - 0.002,
        (b_sa["mid_y"] + b_desc["mid_y"]) / 2,
        "meshes symlink",
        fontsize=7,
        color="#1a5276",
        ha="right",
        va="center",
        rotation=90,
        zorder=6,
    )

    # bringup -> runtime packages (short dotted upward)
    for b in (b_control, b_fov, b_hw, b_gzpkg):
        arrow(ax, b_bringup["cx"], b_bringup["top"], b["cx"], b["bot"], color="#1e8449", style=":", lw=1.0)

    # Legend
    ax.add_patch(
        FancyBboxPatch(
            (0.04, 0.012),
            0.92,
            0.080,
            boxstyle="round,pad=0.01,rounding_size=0.012",
            facecolor="#ffffff",
            edgecolor="#aab7b8",
            lw=1.0,
            zorder=3,
        )
    )
    ax.text(0.06, 0.078, "主路径　浏览器 → web → rosbridge → control / fov / hw → Gazebo 或真机", fontsize=8.5, va="top", color="#1c2833", zorder=4)
    ax.text(0.06, 0.055, "蓝虚线　sensors-arm 复用（UI / 配置 / 网格）；驱动只在 hik-sensors", fontsize=8.5, va="top", color="#1a5276", zorder=4)
    ax.text(0.06, 0.032, "绿点线　bringup 只组装 launch　·　msgs 为共享接口", fontsize=8.5, va="top", color="#1e8449", zorder=4)

    png = OUT / "architecture.png"
    svg = OUT / "architecture.svg"
    fig.savefig(png, bbox_inches="tight", facecolor=fig.get_facecolor())
    fig.savefig(svg, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"wrote {png}")
    print(f"wrote {svg}")


if __name__ == "__main__":
    main()
