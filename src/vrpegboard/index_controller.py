"""Valve Index controller charging dock (right hand; left = mirror on export).

The Index controller is a genuinely awkward shape, but the references (see the
README — esp. thing:3728198) show the clean solution: a rectangular **holder
block with the controller's own bottom subtracted out of it**, so the controller
nests into a depression that exactly matches its base. No upper retaining ring —
the magnetic cable does most of the holding, the contoured pocket registers the
controller and stops it swinging, and gravity keeps it seated.

So the holder is ``box - controller``: a block (minus a slightly enlarged copy of
the controller, for print clearance) into which the base nests and drops straight
in from the top, resting on full contact. The block is **axis-aligned to the board
and extends solidly back to the board face**, with the leaned controller's port
standing off so its body clears the board and solid material filling the gap behind
it. The modelled USB-C jack is **drilled out** (the subtraction would otherwise
leave a plug in the receptacle), and the cable clamp hangs below it.

Held dead-vertical the controller would press into the board, so the whole holder
(and the controller pocket in it) is **angled outward**: ``MOUNT_ANGLE`` (``None``
auto-solves the smallest angle that clears the board; set a number to fix it).
``STANDOFF`` sets how far the port stands off the board (and trades against lean).

Auto-pose from the STEP: find the button "head" vs the handle "pommel/port" end,
derive the handle axis, rotate it vertical with the **USB-C port facing down**,
and spin the ring forward so it clears the board. The port is assumed centred on
the handle bottom — CONFIRM against your controller.
"""

from __future__ import annotations

from functools import lru_cache
from math import acos, atan2, degrees

import manifold3d as m3
import numpy as np
import trimesh
from build123d import (
    Box,
    Cylinder,
    Location,
    Part,
    Pos,
    Rot,
    Vector,
)

from .connector import connector_clamp
from .cradle import apply_location, grip_center, lean_angle, lean_loc, ocp_cloud
from .mesh import manifold_to_trimesh, swept, to_manifold
from .params import BACKPLATE, CONNECTOR, PEGBOARD
from .pegboard import peg_back

STEP = "vendor/index_controller.stp"

# --- Tuning knobs (canonical coords: port at z=0, handle vertical, head at +Z) --
CUP_DEPTH = 30.0  # how much of the controller's base the holder wraps (the bottom ~3 cm).
#                   Keep it in the straighter handle/lower-ring region so the pocket
#                   has no undercut and the controller drops straight in from the top.
CUP_WALL = 5.0  # block wall thickness around the carved pocket
CUP_CLEARANCE = 1.0  # radial gap between the controller and the pocket (drop-in slide)
JACK_DRILL_DIA = 8.0  # bore up the controller axis to clear the modeled USB-C receptacle
#                       (the subtraction would otherwise leave a plug of material in it)
#                       and to house the rigid magnetic body above the cable clamp.

# The USB-C port frame in canonical coords. Valve's STEP models the port only as a
# recess, and on the real controller it is both off-centre AND tilted off the handle
# axis, so we pin it explicitly. Mark a rectangle on the port face in FreeCAD, export it
# to STEP, and read these off it with ``portmark.port_frame_from_marked_step``.
#   PORT_XY   : (x, y) of the port-face centre
#   PORT_Z    : z of the port mate face
#   PORT_AXIS : outward port normal (the way the port faces / the cable points)
# Defaults (None) fall back to the handle-bottom centroid down the handle axis — the old
# guess, which is wrong; they exist only so the module imports before the port is marked.
# Derived from a rectangle marked on the port face in FreeCAD (vendor/
# indexcontroller-BodyChargingPortRect.step + .3mf) via
# ``portmark.port_frame_from_marked_step``. The port is off-centre and tilts ~20° off the
# handle axis (cable points down-and-outward), which the old centroid-down-the-handle guess
# couldn't capture — re-run portmark and paste here if the marked rectangle changes.
PORT_XY: tuple[float, float] | None = (-15.47, -5.38)
PORT_Z: float = 3.90
PORT_AXIS: tuple[float, float, float] | None = (-0.284, -0.1818, -0.9414)

MOUNT_ANGLE: float | None = None  # outward tilt (deg); None = auto-solve board clearance
BOARD_CLEARANCE = 5.0  # min gap the leaning controller keeps off the board face
# Standoff trades against lean: the handle's back sits ~17 mm behind the port, so a
# small standoff forces a steep lean to clear the board. ~12 mm gives a gentle ~22°
# lean (raise it toward ~18 for a near-vertical hold further off the board).
STANDOFF = 15.0  # how far the port stands off the plate face (+Y)
ATTACH_Z = -PEGBOARD.pitch / 2  # board Z the port sits at (between the two pegs)


@lru_cache(maxsize=1)
def _load_raw() -> Part:
    from build123d import import_step

    return import_step(STEP)


@lru_cache(maxsize=1)
def _canonical_tf() -> Location:
    """The transform ``T`` taking the raw STEP into the canonical pose (``canonical = T*raw``).

    Exposed so geometry marked in the controller's native frame — e.g. the port rectangle
    drawn in FreeCAD (see ``portmark.port_frame_from_marked_step``) — can be mapped into
    canonical coords with the *same* transform, instead of re-deriving the pose by hand.
    """
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

    # Build the transform incrementally, composing each step into ``t`` (all are global
    # pre-multiplies: ``shape.moved(L)`` == ``L * shape``), so ``t`` ends up == the whole pose.
    l1 = Location(Vector(0, 0, 0), Vector(*rot_axis), ang)
    t = l1
    dev = l1 * dev
    # Drop the lowest point (the port) to z=0 and centre the base band on xy origin.
    q = ocp_cloud(dev.wrapped)
    l2 = Location(Vector(0, 0, -q[:, 2].min()))
    t = l2 * t
    dev = l2 * dev
    grip = ocp_cloud(dev.wrapped)
    band = grip[(grip[:, 2] >= 0) & (grip[:, 2] <= CUP_DEPTH)]
    cx, cy, _ = grip_center(band[:, :2])
    l3 = Location(Vector(-cx, -cy, 0))
    t = l3 * t
    dev = l3 * dev

    # Azimuth: spin about the vertical so the head/ring leans toward +Y (the room).
    pts = ocp_cloud(dev.wrapped)
    head = pts[pts[:, 2] > np.percentile(pts[:, 2], 85)]
    hx, hy = head[:, 0].mean(), head[:, 1].mean()
    spin = 90.0 - degrees(atan2(hy, hx))
    return Rot(0, 0, spin) * t


@lru_cache(maxsize=1)
def _canonical() -> Part:
    """Controller posed: handle vertical, port-down at z=0, base footprint centred."""
    return _canonical_tf() * _load_raw()


@lru_cache(maxsize=1)
def _port_xy() -> tuple[float, float]:
    """The port centre in canonical xy — the marked ``PORT_XY``, or the old centroid guess."""
    if PORT_XY is not None:
        return PORT_XY
    cloud = ocp_cloud(_canonical().wrapped)
    low = cloud[cloud[:, 2] < np.percentile(cloud[:, 2], 5)]
    return float(low[:, 0].mean()), float(low[:, 1].mean())


def _port_pt() -> tuple[float, float, float]:
    """The port-face centre in canonical (x, y, z)."""
    x, y = _port_xy()
    return (x, y, PORT_Z)


def _port_axis() -> np.ndarray:
    """Unit outward port normal in canonical coords (defaults straight down the handle)."""
    a = np.array([0.0, 0.0, -1.0]) if PORT_AXIS is None else np.array(PORT_AXIS, float)
    return a / np.linalg.norm(a)


def _port_loc() -> Location:
    """Canonical placement at the port, local +Z pointing **into** the controller.

    The connector clamp is modelled with +Z toward the seated device and the jack drill
    bores along +Z; mapping +Z to ``-PORT_AXIS`` (into the body, opposite the outward
    port normal) keeps both aligned with a port that's tilted off the handle axis.
    """
    px, py, pz = _port_pt()
    into = -_port_axis()
    z = np.array([0.0, 0.0, 1.0])
    ax = np.cross(z, into)
    s = float(np.linalg.norm(ax))
    if s < 1e-9:  # already (anti)parallel to +Z
        rot = Location() if into[2] > 0 else Location(Vector(0, 0, 0), Vector(1, 0, 0), 180)
    else:
        ang = degrees(acos(np.clip(float(z @ into), -1, 1)))
        rot = Location(Vector(0, 0, 0), Vector(*(ax / s)), ang)
    return Pos(px, py, pz) * rot


def _enlarged(dev: Part) -> Part:
    """The controller grown by ~CUP_CLEARANCE radially, about the base (z=0) centre.

    Uniform scale about the base point keeps the underside flush (it rests on the
    pocket floor) while opening a clearance gap up the sides for the drop-in slide.
    The canonical pose already centres the base band on xy=0 at z=0, so scaling
    about the origin grows it about the base.
    """
    pts = ocp_cloud(dev.wrapped, 0.7)
    band = pts[(pts[:, 2] >= 0) & (pts[:, 2] <= CUP_DEPTH)]
    rad = float(np.median(np.hypot(band[:, 0], band[:, 1])))
    factor = (rad + CUP_CLEARANCE) / rad if rad > 0 else 1.0
    return dev.scale(factor)


def _holder() -> Part:
    """The holder block + cable clamp + jack drill, **already placed** in board coords.

    The block is **axis-aligned to the board** and extends solidly back to the board
    face (Y=0); the controller — leaned out so its body clears the board — is
    subtracted to form the drop-in pocket (thing:3728198 style), so the port ends up
    standing off the board with solid material filling the gap behind it. The USB-C
    jack is drilled out (the model's receptacle would otherwise leave a plug), and
    the cable clamp hangs below, its entry slot venting in open air.
    """
    place = _place()
    canon = _canonical()
    px, py = _port_xy()

    # Board-frame extents of the base band's handle cluster (ring outliers clipped),
    # so the axis-aligned block hugs the handle while the ring sticks out the top.
    cpts = ocp_cloud(canon.wrapped, 0.5)
    band = cpts[(cpts[:, 2] >= 0) & (cpts[:, 2] <= CUP_DEPTH)]
    cx, cy, r = grip_center(band[:, :2])
    near = band[np.hypot(band[:, 0] - cx, band[:, 1] - cy) <= r * 1.15]
    b = apply_location(place, near)  # where the band lands in board coords

    # Cable clamp at the port (leaned), gripping the cable below the rigid body, aligned
    # to the (possibly tilted) port axis via _port_loc().
    clamp = place * _port_loc() * connector_clamp(slot_dir=(0.0, 1.0))
    cbb = clamp.bounding_box()

    x0, x1 = float(b[:, 0].min()) - CUP_WALL, float(b[:, 0].max()) + CUP_WALL
    z1 = float(b[:, 2].max()) + CUP_WALL  # top: the band's high edge (controller exits above)
    z0 = cbb.max.Z - 1.0  # bottom: down to the clamp top so they fuse (clamp grip vents below)
    y1 = float(b[:, 1].max()) + CUP_WALL  # front (room side); back face sits on the board (Y=0)
    block = Pos((x0 + x1) / 2, y1 / 2, (z0 + z1) / 2) * Box(x1 - x0, y1, z1 - z0)

    # Drill along the port axis: clears the receptacle plug and houses the rigid body
    # down to the clamp bore. Spans [-male_body_len, +12] along the port axis (+Z local).
    lo, hi = -CONNECTOR.male_body_len, 12.0
    drill = place * _port_loc() * Pos(0, 0, (lo + hi) / 2) * Cylinder(JACK_DRILL_DIA / 2, hi - lo)

    # Drop-in pocket: subtract the controller **swept along its insertion axis**, not the
    # raw solid. Sweeping fills the controller's open concavities (the USB-C recess, the
    # strap groove) so no block material floats inside the pocket, and opens the cavity
    # straight along the axis so the controller drops in from the top. OCC can't fuse this
    # swept volume on the ~390k-triangle import (it times out), so the pocket — and the
    # whole holder — is assembled in mesh space with manifold3d (sub-millisecond). See
    # ``mesh.swept``.
    pocket_solid = place * _enlarged(canon)  # the leaned, enlarged controller
    axis = _insertion_dir(place)  # +controller axis (port->head) in board coords
    travel = (z1 - z0) + CUP_DEPTH  # sweep clears the full block height plus the band
    cavity = swept(to_manifold(pocket_solid), axis, travel, n=28)

    return (to_manifold(block) - cavity - to_manifold(drill)) + to_manifold(clamp)


def _insertion_dir(place: Location) -> tuple[float, float, float]:
    """The controller's +axis (canonical +Z, port->head) expressed in board coords.

    The controller is inserted along its own long axis; leaned out, that axis is the
    canonical +Z rotated by ``place``. Sweeping the pocket along it opens the drop-in path.
    """
    base = apply_location(place, np.array([[0.0, 0.0, 0.0]]))[0]
    tip = apply_location(place, np.array([[0.0, 0.0, 1.0]]))[0]
    d = tip - base
    return tuple(d / np.linalg.norm(d))


def _angle() -> float:
    """Outward tilt: the knob, or the auto-solved minimum clear angle."""
    if MOUNT_ANGLE is not None:
        return MOUNT_ANGLE
    pts = ocp_cloud(_canonical().wrapped, 0.7)
    return lean_angle(pts, _port_pt(), _port_board_y(), board_clear=BOARD_CLEARANCE)


def _port_board_y() -> float:
    return BACKPLATE.thickness + STANDOFF


def _place() -> Location:
    """Map canonical (port at _port_pt) into board coords: lean out, then position."""
    px, py, pz = _port_pt()
    # Lean about the port, then translate the port to (0, port_board_y, ATTACH_Z).
    return Pos(-px, _port_board_y() - py, ATTACH_Z - pz) * lean_loc(_angle(), _port_pt())


def seated_device() -> Part:
    """The controller posed where it sits in the dock (for preview overlays)."""
    return _place() * _canonical()


def _dock_manifold() -> m3.Manifold:
    """The right-hand dock as a single ``manifold3d`` solid (peg-back + holder)."""
    # _holder() is already in board coords (block axis-aligned, reaching the board),
    # so it fuses straight onto the peg-back with no separate spine.
    return to_manifold(peg_back()) + _holder()


def dock() -> trimesh.Trimesh:
    """The right-hand controller dock as a printable mesh, positioned on the convention.

    Returns a ``trimesh.Trimesh`` rather than a build123d ``Part``: the controller
    booleans run in mesh space (manifold3d), and STL is a mesh anyway. ``build.py`` and
    the previews handle either representation via ``mesh.export_solid``/``bbox_size``.
    """
    return manifold_to_trimesh(_dock_manifold())


def dock_left() -> trimesh.Trimesh:
    """Left-hand dock: mirror the right-hand dock across the board's YZ plane (x -> -x)."""
    return manifold_to_trimesh(_dock_manifold().mirror((1.0, 0.0, 0.0)))


def dock_right() -> trimesh.Trimesh:
    return dock()


def min_clear_angle() -> float:
    """Report the smallest outward tilt that clears the board (the auto-solve value)."""
    pts = ocp_cloud(_canonical().wrapped, 0.7)
    return lean_angle(pts, _port_pt(), _port_board_y(), board_clear=BOARD_CLEARANCE)
