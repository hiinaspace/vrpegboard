"""Generic dock-geometry helpers shared by the device modules.

Strategy: pose the imported device so its charging port faces **down** (-Z),
project the relevant band of its surface along the insertion axis into a 2-D
**silhouette** outline (shapely), and extrude that into drop-in pockets/cups.
Lean solves (the outward tilt vs the standoff off the board) work on tessellated
point clouds of the posed device.

Posing (rotation + where the port sits) is device-specific and lives in the
device modules; those values are confirmed visually via ``preview``/``fitcheck``.
"""

from __future__ import annotations

import numpy as np
from build123d import Box, Location, Part, Pos, Rot
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


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


def lean_standoff(
    pts: np.ndarray,
    pivot: tuple[float, float, float],
    angle_deg: float,
    board_clear: float = 2.0,
) -> float:
    """Board-Y the pivot (port) must stand at so the device, leaned ``angle_deg``
    out about the pivot, keeps every point at least ``board_clear`` off the board.

    The inverse of ``lean_angle``: there the standoff is fixed and the lean is
    solved; here the lean is a chosen, intentional-looking angle and the standoff
    absorbs the clearance. Closed form — the leaned cloud's most-negative relative
    Y sets it directly.
    """
    p = pts - np.asarray(pivot, dtype=float)
    rad = np.radians(angle_deg)  # lean +Y: Y' = y*cos + z*sin (top z>0 -> +Y)
    y_rel = p[:, 1] * np.cos(rad) + p[:, 2] * np.sin(rad)
    return board_clear - float(y_rel.min())


def lean_rest_angle(
    pts: np.ndarray,
    pivot: tuple[float, float, float],
    port_board_y: float,
    board_clear: float = 0.5,
    min_angle: float = -30.0,
    step: float = 0.25,
) -> float:
    """Most negative (toward-board) lean that keeps ``board_clear`` off the board.

    The opposite intent of ``lean_angle``: instead of tipping *out* until the
    device clears, tip it *back* until it almost touches — the device then rests
    against the board surface, which steadies it, instead of hanging angled out
    into the room. Returns 0.0 if even vertical doesn't clear (raise the standoff).
    """
    p = pts - np.asarray(pivot, dtype=float)
    a, best = 0.0, 0.0
    while a >= min_angle:
        rad = np.radians(a)
        y_rel = p[:, 1] * np.cos(rad) + p[:, 2] * np.sin(rad)
        if float(y_rel.min()) + port_board_y < board_clear:
            break
        best = a
        a -= step
    return round(best, 2)


def tapered_base(outline, z_top: float, z_bot: float, center_xy, r_min: float) -> Part:
    """A loft from ``outline`` (at ``z_top``) down to a scaled copy at ``z_bot``.

    The body below a dock's pocket floor only has to carry the connector socket,
    so instead of extruding the full silhouette straight down, taper it toward
    the socket: scale the outline about ``center_xy`` (the socket's bottom axis)
    just enough that the bottom still wraps ``r_min`` of material around the
    socket and the walls stay ≤45° from vertical (so the upright print needs no
    support). Scaling the *same* ring gives the loft a clean 1:1 vertex
    correspondence — a smooth ruled surface, no twist.
    """
    import shapely
    from build123d import loft
    from shapely import affinity

    cx, cy = float(center_xy[0]), float(center_xy[1])
    coords = np.asarray(outline.exterior.coords)
    d = np.hypot(coords[:, 0] - cx, coords[:, 1] - cy)
    d_min = outline.exterior.distance(shapely.Point(cx, cy))
    h = z_top - z_bot
    s = max(r_min / max(d_min, 1e-6), 1.0 - h / float(d.max()))
    if s >= 1.0:  # no room to taper; plain prism
        from build123d import extrude

        return extrude(poly_face(outline, z_bot), amount=h, dir=(0, 0, 1))
    bottom = affinity.scale(outline, xfact=s, yfact=s, origin=(cx, cy))
    return loft([poly_face(bottom, z_bot), poly_face(outline, z_top)], ruled=True)


def silhouette(
    xy: np.ndarray, closing: float = 4.0, relief: float = 0.8, simplify_tol: float = 0.25
):
    """Outer outline (shapely Polygon/MultiPolygon) of an Nx2 surface-point set.

    The points are a device's tessellation nodes within a Z band, projected along
    the insertion axis; the returned outline is the cross-section a straight
    drop-in pocket must clear. The nodes lie on *surfaces*, so the projection is
    an annulus-ish shell with sparse stretches (no interior fill) — a concave
    hull snakes into the voids, so instead use a **morphological closing**:
    dilate every point by ``closing`` (bridging gaps up to twice that — pick it
    above half the largest node spacing), erode back, and fill each part's
    interior (a void inside the outline is still occupied by the physical
    device). The erosion is held back by ``relief``: a full erosion thins
    one-node-wide fringes — exactly the silhouette-grazing nodes — to nothing,
    so the boundary instead stays that margin outside every node.

    ``closing`` doubles as the smoothing knob: bigger bridges more surface detail
    (screw bosses, seams) into a calmer outline — the printed walls look
    intentional instead of wiggly — at the cost of slightly overcovering concave
    features narrower than twice the radius.
    """
    import shapely

    if len(xy) > 80_000:  # cap the per-point buffering cost; don't go lower — the
        # erosion relief only protects points actually kept, and aggressive
        # subsampling visibly cuts the outline through sparse fringe chains
        xy = xy[:: len(xy) // 80_000 + 1]
    blob = shapely.MultiPoint(xy).buffer(closing).buffer(-(closing - relief))
    parts = [shapely.Polygon(g.exterior) for g in getattr(blob, "geoms", [blob])]
    return shapely.unary_union(parts).simplify(simplify_tol).buffer(0)


def poly_face(poly, z: float = 0.0) -> Part:
    """A build123d planar face (at height ``z``) from a shapely Polygon's exterior."""
    from build123d import Face, Vector, Wire

    pts = [Vector(x, y, z) for x, y in poly.exterior.coords[:-1]]
    return Face(Wire.make_polygon(pts, close=True))


def above_plane(point: tuple[float, float, float], normal: tuple[float, float, float]) -> Part:
    """A big half-space stand-in (500 mm box) covering the +normal side of a plane.

    Subtract it to cut a solid flush along an arbitrary plane (e.g. the pocket rim
    plane perpendicular to the leaned insertion axis).
    """
    from build123d import Plane

    size = 500.0
    pl = Plane(origin=point, z_dir=normal)
    return pl * Pos(0, 0, size / 2) * Box(size, size, size)


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
