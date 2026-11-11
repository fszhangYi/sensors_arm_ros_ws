"""Pinhole projection FOV checks (CameraInfo primary path).

Compatible with Python 3.8 (ROS 2 Foxy).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero vector")
    return v / n


def fov_from_intrinsics(width: float, height: float, fx: float, fy: float) -> Tuple[float, float]:
    h = math.degrees(2.0 * math.atan(float(width) / (2.0 * float(fx))))
    v = math.degrees(2.0 * math.atan(float(height) / (2.0 * float(fy))))
    return h, v


def intrinsics_from_fov(width: float, height: float, h_deg: float, v_deg: float) -> Tuple[float, float]:
    fx = float(width) / (2.0 * math.tan(math.radians(float(h_deg) / 2.0)))
    fy = float(height) / (2.0 * math.tan(math.radians(float(v_deg) / 2.0)))
    return fx, fy


def resolve_mode_optics(mode: Dict[str, Any]) -> Dict[str, float]:
    """Return width, height, fx, fy, H_deg, V_deg for a mode dict."""
    w = float(mode.get("width") or 1280)
    h = float(mode.get("height") or 720)
    if mode.get("H_deg") is not None and mode.get("V_deg") is not None:
        H = float(mode["H_deg"])
        V = float(mode["V_deg"])
        fx, fy = intrinsics_from_fov(w, h, H, V)
    elif mode.get("fx") is not None and mode.get("fy") is not None:
        fx = float(mode["fx"])
        fy = float(mode["fy"])
        H, V = fov_from_intrinsics(w, h, fx, fy)
    else:
        raise ValueError("mode needs H_deg+V_deg or fx+fy")
    return {"width": w, "height": h, "fx": fx, "fy": fy, "H_deg": H, "V_deg": V}


def axes_from_cfg(cfg: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    lens = np.asarray(cfg["lens"], dtype=float)
    look = unit(np.asarray(cfg["look"], dtype=float))
    right = unit(np.asarray(cfg["right"], dtype=float))
    if cfg.get("up") is not None:
        up = unit(np.asarray(cfg["up"], dtype=float))
    else:
        up = unit(np.cross(look, right))
    right = unit(np.cross(up, look))
    return lens, look, right, up


def project_angular(
    p: np.ndarray, lens: np.ndarray, look: np.ndarray, right: np.ndarray, up: np.ndarray
) -> Tuple[float, float, float]:
    rel = p - lens
    d = float(rel @ look)
    az = float(np.degrees(np.arctan2(rel @ right, d)))
    el = float(np.degrees(np.arctan2(rel @ up, d)))
    return d, az, el


def project_pixel(
    p_optical: np.ndarray, fx: float, fy: float, cx: float, cy: float
) -> Tuple[float, float, float]:
    """Project 3D point in optical frame (Z forward) to pixel; returns u,v,Z."""
    z = float(p_optical[2])
    if abs(z) < 1e-9:
        return float("nan"), float("nan"), z
    u = fx * float(p_optical[0]) / z + cx
    v = fy * float(p_optical[1]) / z + cy
    return u, v, z


def in_image(u: float, v: float, z: float, width: float, height: float, min_z: float = 1e-3) -> bool:
    if not (z > min_z):
        return False
    if math.isnan(u) or math.isnan(v):
        return False
    return 0.0 <= u < float(width) and 0.0 <= v < float(height)


def tool_to_optical_matrix(look: np.ndarray, right: np.ndarray, up_fov: np.ndarray) -> np.ndarray:
    """3x3 rotation: p_optical = R.T @ (p_tool - lens) with columns [right, Y_down, look]."""
    y_down = unit(np.cross(look, right))
    R = np.column_stack([right, y_down, look])  # tool basis of optical axes
    return R.T  # maps tool vectors -> optical


def analyze_projection(
    cfg: Dict[str, Any],
    *,
    mode_overrides: Optional[Dict[str, Any]] = None,
    tf_points_optical: Optional[Dict[str, np.ndarray]] = None,
) -> Dict[str, Any]:
    """Primary FOV path: project samples with mode intrinsics.

    If ``tf_points_optical`` is provided (name -> xyz meters in optical frame),
    use those positions; else transform samples from tool mm via lens/look/right.
    """
    lens, look, right, up = axes_from_cfg(cfg)
    R_to_opt = tool_to_optical_matrix(look, right, up)

    samples = list(cfg.get("samples") or [])
    modes_in = list(cfg.get("modes") or [])
    if mode_overrides:
        # Put override mode first for primary reporting
        ov = dict(mode_overrides)
        if "id" not in ov:
            ov["id"] = ov.get("id") or "override"
        modes_in = [ov] + [m for m in modes_in if m.get("id") != ov.get("id")]

    points: List[Dict[str, Any]] = []
    for s in samples:
        name = s.get("name", "")
        xyz_mm = np.asarray(s["xyz"], dtype=float)
        if tf_points_optical is not None and name in tf_points_optical:
            p_opt = np.asarray(tf_points_optical[name], dtype=float).reshape(3)
            z = float(p_opt[2])
            d = float(z * 1000.0)
            az = float(np.degrees(np.arctan2(float(p_opt[0]), z))) if abs(z) > 1e-9 else 0.0
            el = float(np.degrees(np.arctan2(-float(p_opt[1]), z))) if abs(z) > 1e-9 else 0.0
        else:
            # Primary path: flange/tool-frame samples → optical via lens/look/right.
            rel_mm = xyz_mm - lens
            p_opt = (R_to_opt @ rel_mm) / 1000.0  # mm → m
            d, az, el = project_angular(xyz_mm, lens, look, right, up)
        points.append(
            {
                "name": name,
                "tag": s.get("tag", ""),
                "label_zh": s.get("label_zh") or name,
                "role_zh": s.get("role_zh") or "",
                "xyz": xyz_mm.tolist(),
                "p_optical_m": p_opt.tolist(),
                "d_mm": d,
                "az_deg": az,
                "el_deg": el,
                "in_fov": {},
                "blocked": {},
                "u_px": {},
                "v_px": {},
                "margin_h_deg": {},
                "margin_v_deg": {},
            }
        )

    mode_rows: List[Dict[str, Any]] = []
    for m in modes_in:
        opt = resolve_mode_optics(m)
        W, Himg = opt["width"], opt["height"]
        fx, fy = opt["fx"], opt["fy"]
        Hdeg, Vdeg = opt["H_deg"], opt["V_deg"]
        cx, cy = W / 2.0, Himg / 2.0
        flags = []
        for pt in points:
            p_opt = np.asarray(pt["p_optical_m"], dtype=float)
            # p_optical from tool transform is in meters; fx expects same units as Z
            u, v, z = project_pixel(p_opt, fx, fy, cx, cy)
            ok = in_image(u, v, z, W, Himg, min_z=0.001)
            mid = m["id"]
            pt["in_fov"][mid] = ok
            pt["u_px"][mid] = u
            pt["v_px"][mid] = v
            pt["margin_h_deg"][mid] = Hdeg / 2.0 - abs(pt["az_deg"])
            pt["margin_v_deg"][mid] = Vdeg / 2.0 - abs(pt["el_deg"])
            flags.append(ok)
        n_ok = sum(1 for f in flags if f)
        hooks_o = [p for p in points if p["tag"] == "hook_o"]
        margin_h = None
        if hooks_o:
            margin_h = Hdeg / 2.0 - max(abs(p["az_deg"]) for p in hooks_o)
        mode_rows.append(
            {
                "id": m["id"],
                "label": m.get("label", m["id"]),
                "width": int(W),
                "height": int(Himg),
                "fx": fx,
                "fy": fy,
                "H_deg": Hdeg,
                "V_deg": Vdeg,
                "half_H": Hdeg / 2.0,
                "half_V": Vdeg / 2.0,
                "in_count": n_ok,
                "n": len(points),
                "open_hook_h_margin_deg": margin_h,
                "all_in": n_ok == len(points) and len(points) > 0,
            }
        )

    recommend = None
    full = [m for m in mode_rows if m["all_in"]]
    if full:
        recommend = max(full, key=lambda m: m["H_deg"] * m["V_deg"])["id"]
    elif mode_rows:
        recommend = max(mode_rows, key=lambda m: (m["in_count"], m["H_deg"]))["id"]
    for m in mode_rows:
        m["recommend"] = m["id"] == recommend

    return {
        "lens": lens.tolist(),
        "look": look.tolist(),
        "right": right.tolist(),
        "up": up.tolist(),
        "points": points,
        "modes": mode_rows,
        "recommend_mode": recommend,
        "frame": cfg.get("frame", ""),
        "units": cfg.get("units", "mm"),
        "camera_model": cfg.get("camera_model", "D405"),
        "notes": cfg.get("notes", ""),
        "method": "camera_info_projection",
    }


def apply_image_occlusion(
    result: Dict[str, Any],
    image_rgb: np.ndarray,
    mode_id: str,
    *,
    patch: int = 3,
    bg_luma_min: float = 200.0,
) -> Dict[str, Any]:
    """Mark blocked if in_fov but neighborhood looks like bright background."""
    if image_rgb is None or image_rgb.size == 0:
        return result
    h, w = image_rgb.shape[:2]
    for pt in result["points"]:
        if not pt["in_fov"].get(mode_id, False):
            pt["blocked"][mode_id] = False
            continue
        u = pt["u_px"].get(mode_id)
        v = pt["v_px"].get(mode_id)
        if u is None or v is None or math.isnan(u) or math.isnan(v):
            pt["blocked"][mode_id] = False
            continue
        ui, vi = int(round(u)), int(round(v))
        x0, x1 = max(0, ui - patch), min(w, ui + patch + 1)
        y0, y1 = max(0, vi - patch), min(h, vi + patch + 1)
        crop = image_rgb[y0:y1, x0:x1]
        if crop.size == 0:
            pt["blocked"][mode_id] = True
            pt["in_fov"][mode_id] = False
            continue
        luma = crop.astype(np.float64).mean(axis=2) if crop.ndim == 3 else crop.astype(np.float64)
        # Bright sky/background → likely not on fingertip geometry
        blocked = bool(float(np.mean(luma)) >= bg_luma_min)
        pt["blocked"][mode_id] = blocked
        if blocked:
            pt["in_fov"][mode_id] = False
    # refresh mode counts
    for m in result["modes"]:
        mid = m["id"]
        flags = [p["in_fov"].get(mid, False) for p in result["points"]]
        m["in_count"] = sum(1 for f in flags if f)
        m["all_in"] = m["in_count"] == m["n"] and m["n"] > 0
    full = [m for m in result["modes"] if m["all_in"]]
    if full:
        rec = max(full, key=lambda m: m["H_deg"] * m["V_deg"])["id"]
    else:
        rec = max(result["modes"], key=lambda m: (m["in_count"], m["H_deg"]))["id"] if result["modes"] else None
    result["recommend_mode"] = rec
    for m in result["modes"]:
        m["recommend"] = m["id"] == rec
    return result


def _encode_png_rgb(image_rgb: np.ndarray) -> bytes:
    """Encode HxWx3 uint8 RGB as PNG bytes (cv2 or PIL)."""
    import io

    if image_rgb is None or image_rgb.size == 0:
        raise ValueError("empty image")
    img = np.asarray(image_rgb)
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    try:
        import cv2  # type: ignore

        ok, buf = cv2.imencode(".png", img[:, :, ::-1])  # RGB → BGR for OpenCV
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return bytes(buf)
    except Exception:
        from PIL import Image  # type: ignore

        bio = io.BytesIO()
        Image.fromarray(img, mode="RGB").save(bio, format="PNG")
        return bio.getvalue()


def _load_zh_font(size: int):
    from PIL import ImageFont  # type: ignore

    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except Exception:
            try:
                return ImageFont.truetype(path, size=size, index=0)
            except Exception:
                continue
    return ImageFont.load_default()


def render_gazebo_overlay_png(
    result: Dict[str, Any],
    image_rgb: np.ndarray,
    mode_id: str,
    *,
    title: Optional[str] = None,
) -> bytes:
    """Draw projected sample points on a Gazebo camera frame → PNG bytes.

    Pixel coords in ``result`` are in the mode's resolution; they are scaled to
    the actual image size when they differ.
    """
    from PIL import Image, ImageDraw  # type: ignore

    img = np.asarray(image_rgb)
    if img.ndim != 3 or img.shape[2] < 3:
        raise ValueError("expected HxWx3 RGB image")
    h, w = int(img.shape[0]), int(img.shape[1])
    mode = None
    for m in result.get("modes") or []:
        if m.get("id") == mode_id:
            mode = m
            break
    if mode is None and (result.get("modes") or []):
        mode = result["modes"][0]
        mode_id = str(mode.get("id") or mode_id)
    mw = float((mode or {}).get("width") or w)
    mh = float((mode or {}).get("height") or h)
    sx = float(w) / mw if mw > 1e-6 else 1.0
    sy = float(h) / mh if mh > 1e-6 else 1.0

    canvas = Image.fromarray(img[:, :, :3].copy(), mode="RGB")
    draw = ImageDraw.Draw(canvas)
    font = _load_zh_font(15)
    font_sm = _load_zh_font(13)
    font_tiny = _load_zh_font(11)

    status_legend = [
        ("在画面内", (46, 204, 113)),
        ("出画了", (231, 76, 60)),
        ("被挡住", (243, 156, 18)),
    ]
    for pt in result.get("points") or []:
        u = pt.get("u_px", {}).get(mode_id)
        v = pt.get("v_px", {}).get(mode_id)
        if u is None or v is None:
            continue
        try:
            uf, vf = float(u), float(v)
        except (TypeError, ValueError):
            continue
        if math.isnan(uf) or math.isnan(vf):
            continue
        ui = int(round(uf * sx))
        vi = int(round(vf * sy))
        in_fov = bool(pt.get("in_fov", {}).get(mode_id, False))
        blocked = bool(pt.get("blocked", {}).get(mode_id, False))
        if blocked:
            color = (243, 156, 18)
        elif in_fov:
            color = (46, 204, 113)
        else:
            color = (231, 76, 60)
        r = 7
        draw.ellipse((ui - r, vi - r, ui + r, vi + r), outline=color, width=2)
        draw.line((ui - 12, vi, ui + 12, vi), fill=color, width=1)
        draw.line((ui, vi - 12, ui, vi + 12), fill=color, width=1)
        label = str(pt.get("label_zh") or pt.get("name") or "")
        draw.text((ui + 10, vi - 8), label, fill=color, font=font_sm)

    in_count = (mode or {}).get("in_count")
    n = (mode or {}).get("n")
    if title:
        hdr = title
    else:
        hdr = "腕部相机能拍到夹具上的哪些点？  档位 {}".format(mode_id)
        if in_count is not None and n is not None:
            hdr = "{} · 入画 {}/{}".format(hdr, in_count, n)
    draw.rectangle((0, 0, w, 44), fill=(20, 24, 30))
    draw.text((8, 4), hdr, fill=(236, 240, 241), font=font)
    draw.text(
        (8, 24),
        "绿点=指尖前端 · 蓝=内侧 · 黄=外侧 · 红方块=根部（不是障碍物）",
        fill=(180, 190, 200),
        font=font_tiny,
    )
    bar_h = 28
    draw.rectangle((0, h - bar_h, w, h), fill=(20, 24, 30))
    x = 8
    y = h - 20
    for name, col in status_legend:
        draw.ellipse((x, y, x + 10, y + 10), fill=col)
        draw.text((x + 14, y - 2), name, fill=(236, 240, 241), font=font_sm)
        x += 78

    bio = __import__("io").BytesIO()
    canvas.save(bio, format="PNG")
    return bio.getvalue()


def build_gazebo_figures(
    result: Dict[str, Any],
    image_rgb: Optional[np.ndarray],
    *,
    primary_mode: Optional[str] = None,
    all_modes: bool = False,
) -> Dict[str, bytes]:
    """Build figure name → PNG bytes from a live Gazebo frame.

    By default only the raw frame + primary/recommend overlay (keeps zip small
    for rosbridge). Set ``all_modes=True`` to render every mode overlay.
    """
    figs: Dict[str, bytes] = {}
    if image_rgb is None or getattr(image_rgb, "size", 0) == 0:
        return figs
    figs["gazebo_frame.png"] = _encode_png_rgb(image_rgb)
    modes = list(result.get("modes") or [])
    prefer = primary_mode or result.get("recommend_mode") or (modes[0]["id"] if modes else None)
    ordered = []
    if prefer:
        ordered.append(str(prefer))
    if all_modes:
        for m in modes:
            mid = str(m.get("id") or "")
            if mid and mid not in ordered:
                ordered.append(mid)
    done = set()
    for mid in ordered:
        if mid in done:
            continue
        try:
            figs["gazebo_overlay_{}.png".format(mid)] = render_gazebo_overlay_png(
                result, image_rgb, mid
            )
            done.add(mid)
        except Exception:
            continue
    return figs


def build_export_zip(
    result: Dict[str, Any],
    cfg: Dict[str, Any],
    arm_id: str = "",
    *,
    figures: Optional[Dict[str, bytes]] = None,
    image_rgb: Optional[np.ndarray] = None,
    primary_mode: Optional[str] = None,
) -> bytes:
    import io
    import json
    import zipfile
    from datetime import datetime, timezone

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    root = "fov_{}_{}".format(arm_id or "arm", stamp)
    figs = dict(figures or {})
    if image_rgb is not None and not figs:
        figs.update(build_gazebo_figures(result, image_rgb, primary_mode=primary_mode))

    lines = [
        "# 腕部相机视场校验报告",
        "",
        "## 一句话说明",
        "",
        "Gazebo 里那些彩色球/块，是夹具上的**检查点**（刀根、刀尖、内外钩），"
        "用来回答：腕部相机某个视场档位下，这些位置能不能拍进画面。"
        "它们不是障碍物，也不代表真实零件颜色。",
        "",
        "- 工位: `{}`".format(arm_id or "—"),
        "- 方法: `{}`".format(result.get("method")),
        "- 推荐档: **{}**".format(result.get("recommend_mode") or "—"),
        "- 位姿来源: **{}**".format(result.get("pose_source") or "yaml_tool"),
        "- Gazebo 图: **{}**".format("有" if figs else "无（当时无相机帧）"),
        "",
        "## 颜色约定",
        "",
        "| 颜色 | 含义 |",
        "| --- | --- |",
        "| 红（方块） | 根部：靠近夹爪，容易被挡住或出画 |",
        "| 绿（圆球） | 前端：指尖最前面（最该拍到） |",
        "| 蓝（圆球） | 内侧：指尖靠里的侧边 |",
        "| 黄（圆球） | 外侧：夹爪再张开时的外侧，考验视场宽度 |",
        "",
        "## 档位",
        "",
        "| id | 分辨率 | H×V | 入画 | 推荐 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for m in result.get("modes") or []:
        lines.append(
            "| {} | {}×{} | {:.1f}×{:.1f} | {}/{} | {} |".format(
                m["id"], m["width"], m["height"], m["H_deg"], m["V_deg"],
                m["in_count"], m["n"], "★" if m.get("recommend") else "",
            )
        )
    lines += [
        "",
        "## 检查点",
        "",
        "| 中文 | 代号 | 说明 | 深度mm |",
        "| --- | --- | --- | --- |",
    ]
    for p in result.get("points") or []:
        lines.append(
            "| {} | {} | {} | {:.1f} |".format(
                p.get("label_zh") or p.get("name", ""),
                p.get("name", ""),
                p.get("role_zh") or p.get("tag", ""),
                p["d_mm"],
            )
        )
    if figs:
        lines += ["", "## 图片", ""]
        for name in sorted(figs.keys()):
            lines.append("- `figures/{}`".format(name))
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("{}/report.md".format(root), "\n".join(lines) + "\n")
        zf.writestr("{}/fov_summary.json".format(root), json.dumps(result, indent=2, ensure_ascii=False))
        zf.writestr("{}/camera_input.json".format(root), json.dumps(cfg, indent=2, ensure_ascii=False))
        for name, data in figs.items():
            zf.writestr("{}/figures/{}".format(root, name), data)
    return buf.getvalue()
