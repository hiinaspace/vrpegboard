"""Mark up the USB-C port on the imported Index controller.

Valve's STEP models the port only as the **negative** of a USB-C connector — a shallow
rectangular recess on the handle bottom — and gives no axis or landmark for it. The
dock's cable clamp and jack drill must sit on that port, so we pin it down explicitly
instead of guessing the handle-bottom centroid (which lands off the real, off-centre
recess, and is what made the old dock misalign).

Workflow — the code-CAD equivalent of eyeballing it in FreeCAD:

1. ``render_markup_views()`` writes orthographic views of the controller in its canonical
   pose (port-down, ``z=0`` at the lowest point), each rendered **full-bleed at a recorded
   millimetre extent** so image pixels map linearly to canonical mm. The *bottom* view is
   a depth heat-map looking up at the underside, where the recess reads as a small patch
   set off in depth from the surrounding face; the *side* views are depth-coloured scatters
   for reading the port's height/tilt.
2. Read the recess's pixel bounding box off the bottom view (→ x, y) and a side view (→ z);
   ``back_project`` converts a pixel bbox to canonical mm and returns its centre + size.
3. Put the centre in ``index_controller.PORT_XY`` / ``PORT_Z`` (and ``PORT_AXIS`` if the
   side views show the recess tilted off the handle axis). ``print_port_block`` formats it.

The image extent equals the full image (the axes fill the figure), so back-projection is
exact: ``x = x0 + (px/W)·(x1-x0)``; ``y = y1 − (py/H)·(y1-y0)`` (image y runs top-down).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from functools import lru_cache  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from build123d import Location, Vector, import_step  # noqa: E402

from .mesh import part_to_trimesh  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "out"

# A view records how to turn pixels back into millimetres.
#   axes: which canonical axes are the image's (horizontal, vertical)
#   extent: (h0, h1, v0, v1) in mm covering the full image
#   size: (W, H) in pixels
PX = 1000  # image is PX×PX, square, so 1 px is constant mm in both directions per view


@lru_cache(maxsize=1)
def _mesh():
    """The canonical controller as a (welded) trimesh, for solid depth rasters."""
    from .index_controller import _canonical

    return part_to_trimesh(_canonical(), tol=0.08)


def _depth_raster(ax, V, faces, h: int, v: int, d: int, near: str = "low") -> None:
    """Solid orthographic depth render via tripcolor, drawn nearest-last (a z-buffer).

    ``h``/``v``/``d`` index the canonical axes used as image horizontal / vertical / depth.
    ``near='low'`` means the viewer is on the −d side (the nearest surface has the smallest
    d, e.g. looking *up* the −Z axis at the underside); colour encodes depth so a recess
    (set off in depth) reads as a distinct patch.
    """
    cd = V[faces][:, :, d].mean(axis=1)
    order = np.argsort(-cd if near == "low" else cd)  # nearest drawn last → wins per pixel
    f = faces[order]
    ax.tripcolor(
        V[:, h], V[:, v], f, facecolors=cd[order], cmap="viridis", shading="flat", linewidth=0
    )


def _square_extent(
    h: np.ndarray, v: np.ndarray, pad: float = 3.0
) -> tuple[float, float, float, float]:
    """A square mm window covering points (h, v) with padding, so px↔mm is isotropic."""
    h0, h1, v0, v1 = h.min() - pad, h.max() + pad, v.min() - pad, v.max() + pad
    cx, cy = (h0 + h1) / 2, (v0 + v1) / 2
    half = max(h1 - h0, v1 - v0) / 2
    return cx - half, cx + half, cy - half, cy + half


def _grid(ax, ex: tuple[float, float, float, float], step: float = 5.0) -> None:
    """Faint mm grid + labelled origin lines, drawn inside the full-bleed axes."""
    x0, x1, y0, y1 = ex
    for gx in np.arange(np.ceil(x0 / step) * step, x1, step):
        ax.axvline(gx, color="k", lw=0.3, alpha=0.12)
        ax.text(gx, y0, f"{gx:.0f}", fontsize=6, alpha=0.5, ha="center", va="bottom")
    for gy in np.arange(np.ceil(y0 / step) * step, y1, step):
        ax.axhline(gy, color="k", lw=0.3, alpha=0.12)
        ax.text(x0, gy, f"{gy:.0f}", fontsize=6, alpha=0.5, ha="left", va="center")
    ax.axhline(0, color="k", lw=0.6, alpha=0.3)
    ax.axvline(0, color="k", lw=0.6, alpha=0.3)


def _save_fullbleed(path: Path, ex: tuple[float, float, float, float], draw) -> dict:
    fig = plt.figure(figsize=(PX / 100, PX / 100), dpi=100)
    ax = fig.add_axes((0, 0, 1, 1))  # axes fill the figure: image extent == data extent
    ax.set_xlim(ex[0], ex[1])
    ax.set_ylim(ex[2], ex[3])
    draw(ax)
    _grid(ax, ex)
    ax.set_xticks([])
    ax.set_yticks([])
    fig.savefig(path, dpi=100)
    plt.close(fig)
    print(f"  -> {path.name}  extent(mm)={tuple(round(v, 1) for v in ex)}  px=({PX},{PX})")
    return {"extent": ex, "size": (PX, PX)}


def render_markup_views(
    out_dir: Path = OUT, z_cap: float = 12.0, handle_radius: float = 22.0
) -> dict[str, dict]:
    """Write the bottom depth-map + two side scatters; return each view's px↔mm mapping.

    ``z_cap`` keeps only the handle underside (the ring is high up); ``handle_radius``
    drops stray ring-arc points so the bottom view zooms onto the handle bottom where the
    USB-C recess lives.
    """
    out_dir.mkdir(exist_ok=True)
    m = _mesh()
    V = np.asarray(m.vertices)
    F = np.asarray(m.faces)
    from .index_controller import _port_pt

    cen = V[F].mean(axis=1)  # triangle centroids, for masking to the handle
    gx, gy, _gz = _port_pt()  # current guess, shown as a marker to correct
    views: dict[str, dict] = {}

    # Bottom view: solid depth render of the handle underside, looking up the −Z axis.
    # Restricted to the handle so the recess isn't lost in the whole-controller footprint.
    handle = np.hypot(cen[:, 0] - gx, cen[:, 1] - gy) < handle_radius
    bottom = F[handle & (cen[:, 2] < z_cap)]
    ex = _square_extent(V[bottom][:, :, 0].ravel(), V[bottom][:, :, 1].ravel())

    def draw_bottom(ax):
        _depth_raster(ax, V, bottom, h=0, v=1, d=2, near="low")
        ax.plot(gx, gy, "rx", ms=12, mew=2)
        ax.text(gx, gy, "  guess", color="r", fontsize=8, va="center")

    print("bottom view (look up the -Z axis at the underside; recess = depth patch):")
    views["bottom_xy"] = _save_fullbleed(out_dir / "port_bottom_xy.png", ex, draw_bottom)

    # Side views: solid depth render of the lower handle, to read the port's Z (and tilt).
    col = F[handle & (cen[:, 2] < 50.0)]
    for name, (i, j, d) in {
        "side_yz": (1, 2, 0),  # Y horizontal, Z vertical, depth = X
        "side_xz": (0, 2, 1),  # X horizontal, Z vertical, depth = Y
    }.items():
        exs = _square_extent(V[col][:, :, i].ravel(), V[col][:, :, j].ravel())

        def draw_side(ax, i=i, j=j, d=d):
            _depth_raster(ax, V, col, h=i, v=j, d=d, near="low")

        print(f"{name} (vertical = canonical Z; port near Z=0):")
        views[name] = _save_fullbleed(out_dir / f"port_{name}.png", exs, draw_side)

    return views


def back_project(view: dict, px_bbox: tuple[float, float, float, float]) -> dict:
    """Pixel bbox (left, top, right, bottom) on a view → canonical mm centre + size.

    Image y runs top-down, so vertical mm flips. Returns ``{center, size}`` in the view's
    two axes (mm).
    """
    x0, x1, y0, y1 = view["extent"]
    W, H = view["size"]
    left, top, right, bottom = px_bbox
    hx = lambda px: x0 + (px / W) * (x1 - x0)  # noqa: E731
    vy = lambda py: y1 - (py / H) * (y1 - y0)  # noqa: E731 (top-down)
    h_lo, h_hi = sorted((hx(left), hx(right)))
    v_lo, v_hi = sorted((vy(top), vy(bottom)))
    return {
        "center": ((h_lo + h_hi) / 2, (v_lo + v_hi) / 2),
        "size": (h_hi - h_lo, v_hi - v_lo),
    }


def _read_3mf_transform(path_3mf: str) -> np.ndarray:
    """The 4×3 placement matrix from a FreeCAD .3mf ``<build><item transform>``.

    Rows 0-2 are the local X/Y/Z basis vectors in world coords; row 3 is the translation.
    A local point ``p`` maps to world as ``p @ M[:3] + M[3]`` (3MF row-vector convention).
    FreeCAD's STEP export of a sketch flattens it to its own z=0 plane and loses this; the
    .3mf keeps it, so the two together recover the rectangle's 3-D placement.
    """
    import xml.etree.ElementTree as ET
    import zipfile

    with zipfile.ZipFile(path_3mf) as z:
        root = ET.fromstring(z.read("3D/3dmodel.model"))
    ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    item = root.find(".//m:build/m:item", ns)
    transform = item.get("transform") if item is not None else None
    if transform is None:
        raise ValueError(f"no build/item transform in {path_3mf}")
    return np.array([float(v) for v in transform.split()]).reshape(4, 3)


def _rect_center_local(step_path: str) -> tuple[np.ndarray, int]:
    """Centre of the marked rectangle in the sketch's local plane.

    The export carries the marked outline plus reference geometry and the sketch's X/Y
    axis lines (which run out to ±10⁴). We keep edge vertices near the cluster median,
    dropping those axis/construction outliers, and average them — the outline (a
    straight-edged rectangle for the Index, a rounded USB-C spline for the Tundra) is
    symmetric about the port centre. ``"BSPLINE".endswith("LINE")`` so both match; if
    nothing does, fall back to all vertices.
    """
    s = import_step(step_path)
    pts = []
    for e in s.edges():
        if str(e.geom_type).endswith("LINE"):
            pts += [tuple(e.start_point()), tuple(e.end_point())]
    if not pts:
        pts = [tuple(v) for v in s.vertices()]
    pts = np.array(pts)
    med = np.median(pts, axis=0)
    keep = np.linalg.norm(pts[:, :2] - med[:2], axis=1) < 50.0  # drop the far axis lines
    rect = pts[keep]
    return rect.mean(axis=0), len(rect)


def port_frame_from_marked_step(
    step_path: str,
    placement_3mf: str | None = None,
    to_frame: Location | None = None,
    kind: str = "index",
) -> dict:
    """FreeCAD-marked port rectangle (STEP + .3mf) → port frame, mapped to a target frame.

    The STEP gives the rectangle in the sketch's local plane; the .3mf's build transform
    places that plane in the device's native frame (centre → port position, the
    transform's Z-basis → port normal). Both are mapped through ``to_frame`` and the
    normal flipped **outward** (the way the port faces). ``kind`` picks the target frame
    and the printed paste block:

    * ``"index"`` — ``to_frame`` defaults to ``index_controller._canonical_tf()``; prints
      ``PORT_XY`` / ``PORT_Z`` / ``PORT_AXIS``.
    * ``"tundra"`` — ``to_frame`` defaults to identity (raw STEP coords); prints
      ``PORT_C`` / ``PORT_AXIS_N`` to paste into ``tundra_tracker.py``.
    """
    if placement_3mf is None:
        placement_3mf = str(Path(step_path).with_suffix(".3mf"))
    m = _read_3mf_transform(placement_3mf)
    basis, trans = m[:3], m[3]
    local_c, n_rect = _rect_center_local(step_path)
    native_c = local_c @ basis + trans  # local → device-native
    native_n = basis[2] / np.linalg.norm(basis[2])  # local +Z basis = sketch-plane normal

    if to_frame is None:
        if kind == "index":
            from .index_controller import _canonical_tf

            to_frame = _canonical_tf()
        else:
            to_frame = Location()

    def _map(p):
        q = (to_frame * Location(Vector(float(p[0]), float(p[1]), float(p[2])))).position
        return np.array([q.X, q.Y, q.Z])

    cc = _map(native_c)
    cn = _map(native_c + native_n) - cc
    cn /= np.linalg.norm(cn)
    if cn[2] > 0:  # outward port normal faces down (board -Z-ish) once posed
        cn = -cn

    axis = tuple(round(float(v), 4) for v in cn)
    native = tuple(round(v, 1) for v in native_c)
    print(f"marked rect: {n_rect} cluster verts; native centre {native}")
    if kind == "tundra":
        frame = {"center": (float(cc[0]), float(cc[1]), float(cc[2])), "axis": axis}
        print("\n# paste into tundra_tracker.py:")
        print(f"PORT_C = ({cc[0]:.2f}, {cc[1]:.2f}, {cc[2]:.2f})")
        print(f"PORT_AXIS_N = {axis}")
        return frame
    frame = {"xy": (float(cc[0]), float(cc[1])), "z": float(cc[2]), "axis": axis}
    xy = tuple(round(v, 2) for v in frame["xy"])
    print(f"canonical port: xy={xy} z={frame['z']:.2f} axis={axis}")
    print_port_block(cc[0], cc[1], cc[2], axis)
    return frame


def print_port_block(
    x: float, y: float, z: float, axis: tuple[float, float, float] | None = None
) -> None:
    """Print the lines to paste into ``index_controller.py``."""
    print("\n# paste into index_controller.py:")
    print(f"PORT_XY = ({x:.2f}, {y:.2f})")
    print(f"PORT_Z = {z:.2f}")
    print(f"PORT_AXIS = {axis if axis else None}")


def _tundra_main() -> None:
    """Read a FreeCAD-marked Tundra port (STEP + .3mf) → PORT_C / PORT_AXIS_N block."""
    step = Path(__file__).resolve().parents[2] / "vendor" / "tundratracker-chargingportsketch.step"
    mf = step.with_suffix(".3mf")
    if not step.exists():
        raise SystemExit(f"missing {step}")
    if not mf.exists():
        raise SystemExit(
            f"missing {mf.name} — FreeCAD's STEP export flattens the sketch to its local\n"
            "plane and loses where it sits on the dome. Export the SAME sketch selection a\n"
            "second time as 3MF (File → Export → 3D Manufacturing Format) next to the STEP,\n"
            "then re-run this. (The .3mf <build> transform carries the placement + normal.)"
        )
    port_frame_from_marked_step(str(step), str(mf), kind="tundra")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "tundra":
        _tundra_main()
    else:
        v = render_markup_views()
        print("\nMark the recess bbox in pixels, then e.g.:")
        print("  from vrpegboard.portmark import render_markup_views, back_project")
        print("  v = render_markup_views()")
        print("  back_project(v['bottom_xy'], (left, top, right, bottom))  # -> x,y")
        print("  back_project(v['side_yz'],  (left, top, right, bottom))  # -> ...,z")
