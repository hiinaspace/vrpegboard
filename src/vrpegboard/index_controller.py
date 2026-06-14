"""Valve Index controller charging dock (right hand; left = mirror on export).

The controller hangs **port straight down** — its charging port faces the floor,
so it stands balanced on top of the magnetic connector and its weight presses
straight down the connector axis (compression), with no peeling moment to pop the
magnet. To keep the whole controller clear of the board while it hangs vertical
the connector is stood a long way off the board (~70 mm) on a thin bracket — worth
it for the balance.

A **surface-conforming cup** of the controller's bottom ~20 mm registers it: the
cup's inner surface is a depth raster of the grip (``conform`` ray-casts the posed
mesh looking up the hang axis), so the grip nests on a matching surface instead of
a flat-floored prism. Because a heightfield is single-valued in z the body still
drops straight in with no undercut. The magnetic connector's two discs + barrel
lead-in sink into a ``connector_socket`` in the cup floor (a full-height side slot
lets the cable press in); the barrel + cable hang free below. ``CUP_METHOD`` picks
a ``"solid"`` conforming block or a ``"shell"`` thin conforming skin.

The bracket runs back to a **2×2 peg grid** (two hooks up top, two straight pegs
below — two columns so the long cantilever can't twist), the pegs separate glued
parts (see ``pegboard``).

Pose knobs: ``AZIMUTH`` spins the controller about the vertical hang axis (≈90° so
the trigger faces the board and the wide tracking ring lies along X, to the side);
``STANDOFF`` (``None`` = auto-solve the minimum that clears the board) sets how far
the cup stands off.

Auto-pose from the STEP (``_canonical``): handle vertical, port-down, base centred.
The marked port frame (``PORT_*``; see *Marking the port* in the README) is
off-centre and tilted ~20° off the handle axis, so "port straight down" tips the
handle ~20° off vertical — which is fine, the cup and bracket follow. The dock is
built in mesh space (``manifold3d``) — the conforming cup is a low-poly raster, so
its booleans are cheap, unlike a B-rep boolean against the 390k-tri vendor mesh.
"""

from __future__ import annotations

from functools import lru_cache
from math import acos, atan2, degrees
from typing import Any

import numpy as np
from build123d import Box, Location, Part, Pos, Rot, Vector

from .connector import connector_socket, socket_bottom_z, socket_outer_dia
from .cradle import apply_location, grip_center, ocp_cloud
from .params import BACKPLATE, CONNECTOR, PEGBOARD
from .pegboard import grid_width, peg_holes

STEP = "vendor/index_controller.stp"

# --- Cup / pose knobs (hang frame: port at origin, port axis = -Z) --------------
CUP_DEPTH = 20.0  # bottom band of the controller the cup wraps (≈ the handle base)
CUP_WALL = 3.5  # cup wall thickness
CUP_CLEARANCE = 1.0  # radial gap controller↔cup (drop-in slide)

AZIMUTH = 90.0  # spin about the vertical hang axis: trigger/front faces the board
#                 (-Y) and the wide tracking ring lies along X (to the side), so the
#                 dock reads edge-on against the wall. dock_left mirrors → ring to the
#                 other side. (The ring still leans boardward with the trigger, which is
#                 why the standoff comes out ~65 mm — the measured ~70 mm offset.)
CUP_METHOD = "solid"  # "solid" (conforming block) | "shell" (thin conforming skin)
STANDOFF: float | None = None  # cup board-Y (None = auto-solve min board clearance)
BOARD_CLEARANCE = 4.0  # min gap the hanging controller keeps off the board face
CABLE_SLOT_DIR = (0.0, 1.0)  # board-frame xy the cable entry slot exits (default +Y room)
# Flat print base (under the lower-peg margin); the cup, arms, and backplate all bottom
# here. The seat sits a natural socket-depth above it, so the cup's own floor lands on
# the base (cup translated down to meet the backplate, not thickened up to it).
Z_BASE = -(PEGBOARD.pitch + BACKPLATE.margin)
CUP_Z = Z_BASE - socket_bottom_z()  # board Z of the seat (port); cup floor → Z_BASE

# The USB-C port frame in canonical coords (marked in FreeCAD; see the README's
# *Marking the port*). Off-centre and tilted ~20° off the handle axis.
PORT_XY: tuple[float, float] | None = (-15.47, -5.38)
PORT_Z: float = 3.90
PORT_AXIS: tuple[float, float, float] | None = (-0.284, -0.1818, -0.9414)
# The canonicalisation band is **pinned** (the auto-pose is very sensitive to it,
# and the marked PORT_* are only valid in the band=30 frame). Don't tie to CUP_DEPTH.
CANON_BAND = 30.0


@lru_cache(maxsize=1)
def _load_raw() -> Part:
    from build123d import import_step

    return import_step(STEP)


@lru_cache(maxsize=1)
def _canonical_tf() -> Location:
    """Transform taking the raw STEP into the canonical pose (handle vertical, port-down)."""
    dev = _load_raw()
    pts = ocp_cloud(dev.wrapped)
    z = pts[:, 2]
    pommel = pts[z > np.percentile(z, 95)].mean(0)
    head = pts[z < np.percentile(z, 15)].mean(0)
    axis_v = pommel - head
    axis_v /= np.linalg.norm(axis_v)

    target = np.array([0, 0, -1.0])
    rot_axis = np.cross(axis_v, target)
    n = np.linalg.norm(rot_axis)
    ang = degrees(acos(np.clip(axis_v @ target, -1, 1)))
    rot_axis = rot_axis / n if n > 1e-6 else np.array([1.0, 0, 0])

    l1 = Location(Vector(0, 0, 0), Vector(*rot_axis), ang)
    t = l1
    dev = l1 * dev
    q = ocp_cloud(dev.wrapped)
    l2 = Location(Vector(0, 0, -q[:, 2].min()))
    t = l2 * t
    dev = l2 * dev
    grip = ocp_cloud(dev.wrapped)
    band = grip[(grip[:, 2] >= 0) & (grip[:, 2] <= CANON_BAND)]
    cx, cy, _ = grip_center(band[:, :2])
    l3 = Location(Vector(-cx, -cy, 0))
    t = l3 * t
    dev = l3 * dev

    pts = ocp_cloud(dev.wrapped)
    head = pts[pts[:, 2] > np.percentile(pts[:, 2], 85)]
    spin = 90.0 - degrees(atan2(head[:, 1].mean(), head[:, 0].mean()))
    return Rot(0, 0, spin) * t


@lru_cache(maxsize=1)
def _canonical() -> Part:
    """Controller posed: handle vertical, port-down at z=0, base footprint centred."""
    return _canonical_tf() * _load_raw()


def _port_pt() -> tuple[float, float, float]:
    x, y = PORT_XY if PORT_XY is not None else (0.0, 0.0)
    return (x, y, PORT_Z)


def _port_axis() -> np.ndarray:
    a = np.array([0.0, 0.0, -1.0]) if PORT_AXIS is None else np.array(PORT_AXIS, float)
    return a / np.linalg.norm(a)


@lru_cache(maxsize=1)
def _hang_loc() -> Location:
    """Canonical → hang frame: port to the origin, port axis to -Z, then AZIMUTH spin.

    In the hang frame the controller stands port-down (port axis = board -Z) with
    the port at the origin, so the cup/socket build straight and a single
    translation drops it onto the board.
    """
    a = _port_axis()
    b = np.array([0.0, 0.0, -1.0])
    ax = np.cross(a, b)
    s = float(np.linalg.norm(ax))
    if s < 1e-9:
        rot = Location() if a[2] < 0 else Location(Vector(0, 0, 0), Vector(1, 0, 0), 180)
    else:
        ang = degrees(acos(np.clip(float(a @ b), -1, 1)))
        rot = Location(Vector(0, 0, 0), Vector(*(ax / s)), ang)
    px, py, pz = _port_pt()
    return Rot(0, 0, AZIMUTH) * rot * Pos(-px, -py, -pz)


@lru_cache(maxsize=1)
def _hang() -> Part:
    """The controller in the hang frame (port at origin, port axis -Z)."""
    return _hang_loc() * _canonical()


@lru_cache(maxsize=1)
def _hang_cloud() -> np.ndarray:
    return apply_location(_hang_loc(), ocp_cloud(_canonical().wrapped, 0.5))


@lru_cache(maxsize=1)
def _standoff() -> float:
    """Cup board-Y so the whole hanging controller clears the board.

    Vertical hang: the controller's most-negative-Y point (relative to the port at
    the origin) plus the standoff must stay ``BOARD_CLEARANCE`` off the board, and
    the cup's own board-side wall too.
    """
    if STANDOFF is not None:
        return BACKPLATE.thickness + STANDOFF
    pts = _hang_cloud()
    need = BOARD_CLEARANCE - float(pts[:, 1].min())
    return max(need, BACKPLATE.thickness + 2.0)


def _place() -> Location:
    """Hang frame → board coords: drop the port to (0, standoff, CUP_Z)."""
    return Pos(0.0, _standoff(), CUP_Z)


def seated_device() -> Part:
    """The controller posed where it sits in the dock (for preview overlays)."""
    return _place() * _hang()


def connector_preview() -> tuple[Part, Part]:
    """The mated charging cable + the device's magnetic adapter plate, in board coords
    (for the scene overlay). The seat is the connector's local origin, so a single
    placement drops both onto the dock with the cable head lifted to couple."""
    from .connector import cable_assembly, device_plate

    seat = _place()
    return seat * cable_assembly(), seat * device_plate()


CUP_MAX_R = 26.0  # cup hugs the grip within this radius of the port axis; the tracking
#                   ring loops out past it and exits over the wall (no cup material on it)
WEB_THICK = 6.0  # bracket web thickness (X)
WEB_OFF = 10.0  # the two webs sit at x=±WEB_OFF, straddling the cable that hangs at x=0
PEG_COLS = (-PEGBOARD.pitch / 2, PEGBOARD.pitch / 2)


@lru_cache(maxsize=1)
def _hang_mesh():
    """The posed controller as a triangle soup, for the depth-raster ray-cast."""
    from .mesh import part_to_trimesh

    return part_to_trimesh(_hang(), tol=0.3)


def _floor_z() -> float:
    """Cup floor in the hang frame: the **natural** socket-vent bottom. Placed at
    ``CUP_Z`` it lands on ``Z_BASE``, flush with the backplate/arms — the cup is
    translated down to meet them, not thickened up (which would block the slot)."""
    return socket_bottom_z()


def _cup_manifold(method: str | None = None):
    """Cup body + socket boss, bored for the connector, in the hang frame (Manifold).

    The cup's inner surface conforms to the controller's bottom (``conform`` ray-casts
    the posed mesh, with the installed magnet coupler ``fill``ed in so the hollow port
    doesn't poke up); a solid boss around the bore carries the magnet/barrel; the
    ``connector_socket`` cutter opens the bore and the **full-height** cable slot
    (``slot_top`` at the cup rim) so the cable presses in through the whole body.
    """
    from build123d import Cylinder

    from .conform import conforming_cup
    from .mesh import to_manifold

    method = CUP_METHOD if method is None else method
    floor = _floor_z()
    cup = conforming_cup(
        _hang_mesh(),
        band_hi=CUP_DEPTH,
        floor_z=floor,
        clearance=CUP_CLEARANCE,
        wall=CUP_WALL,
        method=method,
        res=0.7,
        max_r=CUP_MAX_R,
        fill=(CONNECTOR.magnet_dia / 2 + 2.0, CONNECTOR.magnet_depth),
    )
    # Boss around the bore, from the base up to just below the device's lowest surface
    # (never above it, or it'd poke the grip that dips below the seat near the port).
    boss_top = float(_hang_mesh().bounds[0][2]) - CUP_CLEARANCE
    boss = to_manifold(
        Pos(0, 0, (floor + boss_top) / 2) * Cylinder(socket_outer_dia() / 2, boss_top - floor)
    )
    socket = to_manifold(
        connector_socket(slot_dir=CABLE_SLOT_DIR, slot_reach=80.0, slot_top=CUP_DEPTH + 2.0)
    )
    # Cable-clearance vent: barrel + cable hang straight down from below the magnet
    # shelf to the open base. Shroud-width so the magnet still rests on the bore's
    # 8.4→7.0 shelf (a magnet-width vent would shave that shelf off).
    v_top, v_bot = -CONNECTOR.magnet_bore_depth, floor - 2.0
    vent = to_manifold(
        Pos(0, 0, (v_top + v_bot) / 2) * Cylinder(CONNECTOR.shroud_bore / 2, v_top - v_bot)
    )
    return (cup + boss) - socket - vent


def _bracket_manifold():
    """Wide backplate + two gusset webs carrying the cup off the wall (Manifold).

    Each web is a plate in a Y-Z plane at ``x=±WEB_OFF`` (straddling the cable that
    hangs at x=0), full backplate height at the wall and tapering forward to under
    the cup, where its flat top **overlaps the cup's lower solid** (``web_top`` a few
    mm into it) so the arm actually fuses to the cup. The webs stay below the seat,
    so the boardward-leaning controller clears them.
    """
    from build123d import Face, Wire, extrude

    from .mesh import to_manifold

    w = grid_width(PEG_COLS)
    z_top = BACKPLATE.margin
    z_bot = -PEGBOARD.pitch - BACKPLATE.margin
    plate = Pos(0, BACKPLATE.thickness / 2, (z_top + z_bot) / 2) * Box(
        w, BACKPLATE.thickness, z_top - z_bot
    )

    y_wall = BACKPLATE.thickness - 0.5
    y_cup = _standoff() + 3.0  # reach just past the cup's board side
    web_top = CUP_Z  # connect up to the seat (below the device), into the cup's solid
    # Flat bottom at z_bot (= Z_BASE), coplanar with the backplate + cup so the dock
    # sits flat on the bed and prints upright support-free.
    profile = [(y_wall, web_top), (y_cup, web_top), (y_cup, z_bot), (y_wall, z_bot)]
    body = plate
    for cx in (-WEB_OFF, WEB_OFF):
        web = extrude(
            Face(Wire.make_polygon([Vector(cx, y, z) for y, z in profile], close=True)),
            amount=WEB_THICK / 2,
            both=True,
        )
        body = body + web
    return to_manifold(body)


def _cavity_manifold():
    """The drop-in cavity placed in board coords — subtract it so the bracket webs
    (which cross the cup footprint at x=±WEB_OFF) don't fill the controller's space."""
    from .conform import conforming_cavity

    cav = conforming_cavity(
        _hang_mesh(),
        band_hi=CUP_DEPTH,
        floor_z=_floor_z(),
        clearance=CUP_CLEARANCE,
        res=0.7,
        max_r=CUP_MAX_R,
        fill=(CONNECTOR.magnet_dia / 2 + 2.0, CONNECTOR.magnet_depth),
    )
    return cav.translate((0.0, _standoff(), CUP_Z))


def _holder_manifold(method: str | None = None, with_pegs: bool = True):
    """Full dock in board coords (Manifold): placed cup + bracket, cavity/pegs/base cut."""
    from .mesh import to_manifold

    cup = _cup_manifold(method).translate((0.0, _standoff(), CUP_Z))
    body = (cup + _bracket_manifold()) - _cavity_manifold()  # carve webs out of the device space
    if with_pegs:
        body = body - to_manifold(peg_holes(PEG_COLS))
    body = body - to_manifold(Pos(0, -250.0, 0) * Box(600, 500, 600))  # nothing into the board
    body = body - to_manifold(Pos(0, 0, Z_BASE - 250.0) * Box(600, 600, 500))  # flat print base
    return body


def dock(method: str | None = None) -> Any:
    """The right-hand Index dock as one printable mesh (Trimesh)."""
    from .mesh import manifold_to_trimesh

    return manifold_to_trimesh(_holder_manifold(method))


def dock_right(method: str | None = None) -> Any:
    return dock(method)


def dock_left(method: str | None = None) -> Any:
    """Left hand = the right dock mirrored across the YZ plane (ring to the other side)."""
    from .mesh import manifold_to_trimesh

    return manifold_to_trimesh(_holder_manifold(method).mirror((1.0, 0.0, 0.0)))


def cup_test(method: str | None = None) -> Any:
    """Cup + socket alone (no bracket/pegs) — the cheap first print to check the seat,
    mate height, and stability by hand. Flat-bottomed at the socket vent so it stands
    cup-up on the bed."""
    from build123d import Box as _Box

    from .mesh import manifold_to_trimesh, to_manifold

    cup = _cup_manifold(method)
    floor = to_manifold(Pos(0, 0, _floor_z() - 250.0) * _Box(500, 500, 500))
    return manifold_to_trimesh(cup - floor)


def min_clear_angle() -> float:
    """Vertical hang, so 0° — kept for the scene HUD."""
    return 0.0
