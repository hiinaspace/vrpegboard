"""Tundra tracker charging dock — vertical hang, conforming cup, swivel-capture socket.

The tracker hangs **port straight down** (no lean): its weight then presses
straight down the magnetic connector axis (compression, not a peeling moment), so
the magnet stays seated. The port is pinned by a FreeCAD-marked frame (``PORT_C`` /
``PORT_AXIS_N``; see README *Marking the port*) so the magnet bore sits on the real
USB-C centre/normal, and the dome is rolled to the **most compact** orientation
that still clears the board.

Around the bottom of the body sits a **surface-conforming cup**: ``conform``
ray-casts the dome's lower band looking up the hang axis and builds a cup whose
inner surface matches it (so the dome rests on a matching surface and never punches
through the floor — the old two-plane V didn't follow the surface). It only has to
stop the body swinging — the magnet carries the weight. ``CUP_METHOD`` picks a
``"solid"`` conforming block or a ``"shell"`` thin conforming skin.

The magnetic connector's two discs + barrel lead-in drop into a ``connector_socket``
at the cup floor (a full-height side slot lets the cable press in, and a vent bores
the barrel/cable a clear path down through the neck); the barrel + cable hang free
below.

The dock is built in mesh space (``manifold3d``) and prints upright as mounted (flat
bottom on the bed, cup + socket opening up). The pegs are separate glued parts (see
``pegboard``). The three trackers are identical, so one dock STL is printed thrice.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np
from build123d import Box, Location, Part, Pos, Rot, Vector, import_step

from .connector import connector_socket, socket_bottom_z, socket_outer_dia
from .cradle import apply_location, ocp_cloud
from .params import BACKPLATE, CONNECTOR, PEGBOARD
from .pegboard import peg_holes

STEP = "vendor/tundra_tracker.step"
DOME_SOLID = 3  # the dome body (largest solid; the rest is strap plate + discs)

# USB-C port seed window in native dome coords (upper-right of the dome, outer
# wall, Z ~10-21). The exact azimuth is read from the points inside the window.
PORT_AZ = (25.0, 70.0)  # degrees
PORT_Z = (9.0, 22.0)
PORT_R_MIN = 27.0  # outer wall only (the dome is hollow; ignore inner faces)

# Exact port frame in the **raw STEP** (native) coords, marked in FreeCAD (see
# README *Marking the port*). When both are set they pin the magnet bore on the real
# USB-C centre/normal; left ``None`` the seed window above is used (coarse centroid).
PORT_C: tuple[float, float, float] | None = (19.69, 22.21, 14.98)
PORT_AXIS_N: tuple[float, float, float] | None = (-0.6883, -0.7252, -0.0174)

# Flat print base (under the lower-peg margin); the cup, neck, and backplate all bottom
# here. The seat sits a natural socket-depth above it, so the cup's own floor lands on the
# base (cup translated down to meet the backplate, not thickened up to it).
Z_BASE = -(PEGBOARD.pitch + BACKPLATE.margin)
ATTACH_Z = Z_BASE - socket_bottom_z()  # board Z of the seat (port); cup floor → Z_BASE

# V-cradle knobs (posed-local: port at origin, body up +Z, dome toward -Y).
CRADLE_H = 15.0  # how far the V walls rise around the body
CRADLE_WALL = 3.0  # wall thickness outside the V
CRADLE_CLEAR = 1.0  # gap between the body and the V walls
BOARD_CLEAR = 5.0  # min gap the hanging dome keeps off the board face


@lru_cache(maxsize=1)
def _load_raw() -> Part:
    return import_step(STEP)


def _base_from_port(
    port_c: tuple[float, float, float], port_axis: tuple[float, float, float], dome_c: np.ndarray
) -> Location:
    """Native → port frame: marked port axis to board −Z, port to the origin (no roll).

    Aligns the outward port normal (the opening the cable enters) to −Z so the tracker
    hangs port-down, and drops the port to the origin. The roll about the now-vertical
    axis is left free — ``_posed`` picks it to make the dock compact.
    """
    from math import acos, degrees

    a = np.asarray(port_axis, float)
    a /= np.linalg.norm(a)
    if a @ (np.asarray(port_c, float) - dome_c) < 0:  # face outward (the port opening)
        a = -a
    b = np.array([0.0, 0.0, -1.0])
    ax = np.cross(a, b)
    s = float(np.linalg.norm(ax))
    if s < 1e-9:
        rot = Location() if a[2] < 0 else Location(Vector(0, 0, 0), Vector(1, 0, 0), 180)
    else:
        ang = degrees(acos(np.clip(float(a @ b), -1, 1)))
        rot = Location(Vector(0, 0, 0), Vector(*(ax / s)), ang)
    pc = (rot * Location(Vector(*port_c))).position
    return Pos(-pc.X, -pc.Y, -pc.Z) * rot


def _compact_roll(base: Location, dev: Part) -> float:
    """Roll (deg, about the vertical port axis) that holds the tracker closest to the
    board — maximise the smallest body Y, so the standoff that clears it is minimal."""
    pts = apply_location(base, ocp_cloud(dev.wrapped, 0.6))
    th = np.radians(np.arange(0, 360, 2.0))
    # rotated Y of every point at every angle: y' = x·sinθ + y·cosθ
    yr = np.outer(np.sin(th), pts[:, 0]) + np.outer(np.cos(th), pts[:, 1])
    return float(np.arange(0, 360, 2.0)[int(np.argmax(yr.min(axis=1)))])


@lru_cache(maxsize=1)
def _posed() -> Part:
    """The tracker oriented apex(dome) -Y / strap +Y / port -Z, port at the origin.

    Posed-local: the port (outer-wall point) is the origin; the body extends up
    (+Z), the dome hangs toward -Y (the board). Uses the FreeCAD-marked ``PORT_C`` /
    ``PORT_AXIS_N`` when set (exact), else the ``PORT_*`` seed window (coarse).
    """
    dev = _load_raw()
    dome = dev.solids()[DOME_SOLID]
    Dn = ocp_cloud(dome.wrapped, 0.3)
    cx, cy = float(Dn[:, 0].mean()), float(Dn[:, 1].mean())
    dome_c = Dn.mean(0)

    if PORT_C is not None and PORT_AXIS_N is not None:
        base = _base_from_port(PORT_C, PORT_AXIS_N, dome_c)
        return Rot(0, 0, _compact_roll(base, dev)) * base * dev

    x, y, z = Dn[:, 0] - cx, Dn[:, 1] - cy, Dn[:, 2]
    r = np.hypot(x, y)
    th = np.degrees(np.arctan2(y, x))
    seed = (
        (th >= PORT_AZ[0])
        & (th <= PORT_AZ[1])
        & (z >= PORT_Z[0])
        & (z <= PORT_Z[1])
        & (r > PORT_R_MIN)
    )
    pc = Dn[seed].mean(0)
    az = float(np.degrees(np.arctan2(pc[1] - cy, pc[0] - cx)))
    orient = Rot(0, az + 90, 0) * Rot(90, 0, 0) * Pos(-cx, -cy, 0)
    port = (orient * Location((float(pc[0]), float(pc[1]), float(pc[2])))).position
    return Pos(-port.X, -port.Y, -port.Z) * orient * dev


@lru_cache(maxsize=1)
def _standoff() -> float:
    """Board-Y for the port so the whole hanging tracker clears the board.

    Vertical hang (no lean): the most-negative-Y point of the tracker (the dome)
    plus the standoff must stay ``BOARD_CLEAR`` off the board (Y=0).
    """
    pts = ocp_cloud(_posed().wrapped, 0.4)
    need = BOARD_CLEAR - float(pts[:, 1].min())  # port→board-Y so dome clears
    return max(need, BACKPLATE.thickness + 2.0)


def _place() -> Location:
    """Posed-local (port at origin, vertical) → board coords."""
    return Pos(0.0, _standoff(), ATTACH_Z)


def seated_device() -> Part:
    """The tracker posed where it hangs on the magnet (for preview overlays)."""
    return _place() * _posed()


def connector_preview() -> tuple[Part, Part]:
    """The mated charging cable + the tracker's magnetic adapter plate, in board coords
    (for the scene overlay). The seat is the connector's local origin, so a single
    placement drops both onto the dock with the cable head lifted to couple."""
    from .connector import cable_assembly, device_plate

    seat = _place()
    return seat * cable_assembly(), seat * device_plate()


CUP_MAX_R = 22.0  # cup hugs the dome within this radius of the port axis
CUP_METHOD = "solid"  # "solid" (conforming block) | "shell" (thin conforming skin)


@lru_cache(maxsize=1)
def _posed_mesh():
    """The posed tracker as a triangle soup, for the depth-raster ray-cast."""
    from .mesh import part_to_trimesh

    return part_to_trimesh(_posed(), tol=0.3)


def _floor_z() -> float:
    """Cup floor in the posed frame: the **natural** socket-vent bottom. Placed at
    ``ATTACH_Z`` it lands on ``Z_BASE``, flush with the neck/backplate — the cup is
    translated down to meet them, not thickened up (which would block the slot)."""
    return socket_bottom_z()


def _cup_manifold(method: str | None = None):
    """Cup body + socket boss, bored for the connector, in the posed frame (Manifold).

    The cup's inner surface conforms to the dome's lower band (``conform`` ray-casts
    the posed mesh, the magnet coupler ``fill``ed in so the hollow port doesn't poke
    up), so the dome rests on a matching surface and never punches through the floor.
    A boss carries the bore; the ``connector_socket`` cutter opens the bore (breaching
    the floor) and the full-height cable slot, and a shroud-width vent drops the
    barrel/cable to the base while leaving the magnet's 8.4→7.0 rest shelf intact.
    """
    from build123d import Cylinder

    from .conform import conforming_cup
    from .mesh import to_manifold

    method = CUP_METHOD if method is None else method
    mesh = _posed_mesh()
    zf = float(mesh.bounds[0][2])
    floor = _floor_z()
    cup = conforming_cup(
        mesh,
        band_hi=zf + CRADLE_H,
        floor_z=floor,
        clearance=CRADLE_CLEAR,
        wall=CRADLE_WALL,
        method=method,
        res=0.7,
        max_r=CUP_MAX_R,
        fill=(CONNECTOR.magnet_dia / 2 + 2.0, CONNECTOR.magnet_depth),
    )
    boss_top = zf - CRADLE_CLEAR  # up to just below the dome's lowest surface (no poke)
    boss = to_manifold(
        Pos(0, 0, (floor + boss_top) / 2) * Cylinder(socket_outer_dia() / 2, boss_top - floor)
    )
    socket = to_manifold(
        connector_socket(slot_dir=(0.0, 1.0), slot_reach=60.0, slot_top=zf + CRADLE_H + 2.0)
    )
    v_top, v_bot = -CONNECTOR.magnet_bore_depth, floor - 2.0
    vent = to_manifold(
        Pos(0, 0, (v_top + v_bot) / 2) * Cylinder(CONNECTOR.shroud_bore / 2, v_top - v_bot)
    )
    return (cup + boss) - socket - vent


def _body(method: str | None = None):
    """The dock body (conforming cup + neck + backplate, peg holes cut), board coords."""
    from .mesh import to_manifold

    cup = _cup_manifold(method).translate((0.0, _standoff(), ATTACH_Z))

    z_top = BACKPLATE.margin
    cup_bot = ATTACH_Z + _floor_z()  # board z of the cup underside / socket vent
    z0 = min(cup_bot, -PEGBOARD.pitch - BACKPLATE.margin)  # flat-bottom / backplate base
    plate = to_manifold(
        Pos(0, BACKPLATE.thickness / 2, (z_top + z0) / 2)
        * Box(BACKPLATE.width, BACKPLATE.thickness, z_top - z0)
    )
    # Neck bridging the backplate to the standoff cup, below the seat (clear of the
    # dome, which hangs toward the board above z = ATTACH_Z).
    y_wall = BACKPLATE.thickness - 0.5
    neck_w = socket_outer_dia() + 2.0
    neck = to_manifold(
        Pos(0, (y_wall + _standoff() + 2.0) / 2, (ATTACH_Z + cup_bot) / 2)
        * Box(neck_w, (_standoff() + 2.0) - y_wall, ATTACH_Z - cup_bot)
    )

    from build123d import Cylinder

    body = cup + plate + neck
    # The neck/backplate sit on the cable axis, so bore them like the cup: the socket
    # (magnet bore + the barrel shelf) through the seat region, then a shroud-width
    # vent dropping the barrel/cable to the open base (shelf preserved, unlike a
    # magnet-width vent which would shave it).
    seat = Pos(0.0, _standoff(), ATTACH_Z)
    socket = to_manifold(seat * connector_socket(slot_dir=(0.0, 1.0), slot_reach=60.0))
    v_top, v_bot = ATTACH_Z - CONNECTOR.magnet_bore_depth, z0 - 2.0
    vent = to_manifold(
        Pos(0.0, _standoff(), (v_top + v_bot) / 2)
        * Cylinder(CONNECTOR.shroud_bore / 2, v_top - v_bot)
    )
    body = body - socket - vent - to_manifold(peg_holes())
    body = body - _cavity_manifold()  # carve the neck out of the dome's space near the port
    body = body - to_manifold(Pos(0, -250.0, 0) * Box(600, 500, 600))  # nothing into the board
    body = body - to_manifold(Pos(0, 0, z0 - 250.0) * Box(600, 600, 500))  # flat print bottom
    return body


def _cavity_manifold():
    """The drop-in cavity placed in board coords — subtract it so the neck doesn't fill
    the dome's space where it dips below the seat near the port."""
    from .conform import conforming_cavity

    mesh = _posed_mesh()
    zf = float(mesh.bounds[0][2])
    cav = conforming_cavity(
        mesh,
        band_hi=zf + CRADLE_H,
        floor_z=_floor_z(),
        clearance=CRADLE_CLEAR,
        res=0.7,
        max_r=CUP_MAX_R,
        fill=(CONNECTOR.magnet_dia / 2 + 2.0, CONNECTOR.magnet_depth),
    )
    return cav.translate((0.0, _standoff(), ATTACH_Z))


def dock(method: str | None = None):
    """The Tundra dock body as one printable mesh (pegs are separate glued parts)."""
    from .mesh import manifold_to_trimesh

    return manifold_to_trimesh(_body(method))


def min_clear_angle() -> float:
    """Reported for the scene HUD — vertical hang, so always 0°."""
    return 0.0
