"""Generic cradle helpers shared by the device docks.

Strategy: pose the imported device so its charging port faces **down** (-Z), then
build a *band collar* — a block spanning a vertical slice of the device from which
the (slightly enlarged) device solid is subtracted. The result wraps the device's
exact contour at that band with print clearance, automatically catching any flare
(e.g. the Index handle pommel) so it bears weight. A front opening lets the device
push in. The connector pocket is placed at the port so it mates as the device
seats.

Posing (rotation + where the port sits) is device-specific and lives in the
device modules; those values are confirmed visually via ``preview``.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np
from build123d import Box, Location, Part, Pos, Rot, import_step
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

from .params import PRINT


@lru_cache(maxsize=8)
def _load(path: str) -> Part:
    """Import a STEP file (cached — imports are slow)."""
    return import_step(path)


def ocp_cloud(shape, deflection: float = 0.6) -> np.ndarray:
    """Point cloud (Nx3) from OCP face triangulations.

    More robust than ``Part.tessellate`` — some imported solids carry a face
    triangulation build123d's tessellator chokes on. ``shape`` is an OCP
    ``TopoDS_Shape`` (i.e. ``part.wrapped``).
    """
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    pts: list[tuple[float, float, float]] = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                pts.append((p.X(), p.Y(), p.Z()))
        exp.Next()
    return np.array(pts)


def posed(path: str, rot: tuple[float, float, float], port_drop: float = 0.0) -> Part:
    """Return the device rotated by ``rot`` (deg, X/Y/Z) and dropped so its lowest
    point sits at Z = ``port_drop``. Use this so the port end rests at a known Z.
    """
    dev = Rot(*rot) * _load(path)
    bb = dev.bounding_box()
    return Pos(0, 0, port_drop - bb.min.Z) * dev


def apply_location(loc: Location, pts: np.ndarray) -> np.ndarray:
    """Transform an Nx3 point cloud by a build123d ``Location`` (rotation + offset).

    Lets us reason about where a posed/leaned solid's points land in board coords
    without re-tessellating it (e.g. to size an axis-aligned block around the leaned
    device's base band).
    """
    t = loc.wrapped.Transformation()
    rot = np.array([[t.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    off = np.array([t.Value(i, 4) for i in (1, 2, 3)])
    return pts @ rot.T + off


def lean_loc(angle_deg: float, pivot: tuple[float, float, float]) -> Location:
    """A rotation that tilts geometry **away from the board** about the +X axis.

    The board is the XZ plane at Y<=0; +Y is the room. Positive ``angle_deg``
    leans whatever is above the pivot toward +Y (out into the room), about an X
    axis through ``pivot``. Use the same loc on the device, its cup, and the
    connector so they stay mated as the whole assembly tips out.
    """
    px, py, pz = pivot
    # A point above the pivot (+Z) must move to +Y; that needs a *negative* Rot
    # about +X (Y' = -Z*sin), so we negate the user-facing angle here.
    return Pos(px, py, pz) * Rot(-angle_deg, 0, 0) * Pos(-px, -py, -pz)


def lean_angle(
    pts: np.ndarray,
    pivot: tuple[float, float, float],
    port_board_y: float,
    board_clear: float = 2.0,
    max_angle: float = 60.0,
    step: float = 0.5,
) -> float:
    """Smallest outward lean (deg) so the device clears the board.

    ``pts`` is the device point cloud in its canonical (port-down) pose; ``pivot``
    is the port (where it hangs from the magnet); ``port_board_y`` is the board-Y
    the port will sit at once mounted. We tip the cloud out about the pivot until
    no point is closer to the board than ``board_clear`` (board-Y), and return
    that angle (or ``max_angle`` if it never clears within range).
    """
    px, py, pz = pivot
    p = pts - np.array([px, py, pz])
    a = 0.0
    while a <= max_angle:
        rad = np.radians(a)  # lean +Y: Y' = y*cos + z*sin (top z>0 -> +Y)
        y_rel = p[:, 1] * np.cos(rad) + p[:, 2] * np.sin(rad)
        # The port (pivot) ends up at board-Y = port_board_y; everything else is
        # offset from it by its rotated relative-Y.
        if float(y_rel.min()) + port_board_y >= board_clear:
            return round(a, 1)
        a += step
    return max_angle


def grip_center(xy: np.ndarray) -> tuple[float, float, float]:
    """Robust centre + radius of the dominant central cluster in an XY point set.

    Sigma-clips away far outliers (e.g. the Index tracking ring, whose arc shares
    the grip's Z-band) so we size the collar to the grip, not the whole sweep.
    """
    c = np.median(xy, axis=0)
    keep = np.ones(len(xy), dtype=bool)
    for _ in range(6):
        d = np.linalg.norm(xy - c, axis=1)
        thr = d[keep].mean() + 1.0 * d[keep].std()
        keep = d < max(thr, 1.0)
        c = xy[keep].mean(axis=0)
    r = float(np.linalg.norm(xy[keep] - c, axis=1).max())
    return float(c[0]), float(c[1]), r


def band_center(part: Part, z_lo: float, z_hi: float) -> tuple[float, float, float]:
    """Robust XY centre and radius of the device's grip within a Z band."""
    pts = ocp_cloud(part.wrapped)
    sel = pts[(pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)]
    if len(sel) < 3:
        bb = part.bounding_box()
        return (bb.center().X, bb.center().Y, max(bb.size.X, bb.size.Y) / 2)
    return grip_center(sel[:, :2])


def band_collar(
    device: Part,
    z_lo: float,
    z_hi: float,
    *,
    clearance: float | None = None,
    wall: float = 5.0,
    front_opening: float = 0.0,
) -> Part:
    """A collar wrapping ``device`` over [z_lo, z_hi].

    Subtracts a slightly enlarged copy of the device (uniform scale about the band
    centre) to leave a print-clearance gap. ``front_opening`` (mm) cuts a slot in
    +Y so the device can be pushed in from the front.
    """
    clearance = PRINT.fit_clearance if clearance is None else clearance
    cx, cy, r = band_center(device, z_lo, z_hi)
    block_r = r + wall
    h = z_hi - z_lo
    block = Pos(cx, cy, (z_lo + z_hi) / 2) * Box(2 * block_r, 2 * block_r, h)

    # Enlarge the device about the band-centre point so the cavity has clearance
    # (uniform scale; the small vertical component is centred on the band).
    factor = (r + clearance) / r if r > 0 else 1.0
    zmid = (z_lo + z_hi) / 2
    centred = Pos(-cx, -cy, -zmid) * device
    enlarged = Pos(cx, cy, zmid) * centred.scale(factor)
    collar = block - enlarged

    if front_opening > 0:
        slot = Pos(cx, cy + block_r, (z_lo + z_hi) / 2) * Box(front_opening, 2 * block_r, h + 1)
        collar = collar - slot
    return collar
