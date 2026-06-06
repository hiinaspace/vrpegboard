"""Tundra tracker charging dock — magnet-only, no cradle.

The aftermarket magnetic USB-C cable is strong enough to hold an entire tracker
(tested in hand), so this dock is just the **peg-back plus the connector**, with
**no framing around the tracker at all**. The connector is angled outward so the
tracker — hanging on the magnet, dome toward the board, strap plate to the room,
port down — clears the board face instead of pressing into it.

Posing is automatic from the STEP (find the dome solid, seed the USB-C port from
the user's hint, orient apex -Y / strap +Y / port -Z). The only real freedom is
the outward tilt: ``Tundra.mount_angle`` (``None`` auto-solves the smallest angle
that clears the board; set a number to fix it).

The three trackers are identical, so one dock STL is printed three times.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np
from build123d import Box, Location, Part, Pos, Rot, Shape, import_step

from .connector import clamp_outer_dia, connector_clamp
from .cradle import lean_angle, lean_loc, ocp_cloud
from .params import BACKPLATE, PEGBOARD, TUNDRA
from .pegboard import peg_back

STEP = "vendor/tundra_tracker.step"
DOME_SOLID = 3  # the dome body (largest solid; the rest is strap plate + discs)

# USB-C port seed window in native dome coords (from the user's hint: upper-right
# of the dome, on the outer wall, Z ~10-21). We read the exact azimuth from the
# outer-surface points inside this window, so small STEP changes still localise it.
PORT_AZ = (25.0, 70.0)  # degrees
PORT_Z = (9.0, 22.0)
PORT_R_MIN = 27.0  # outer wall only (the dome is hollow; ignore inner faces)

# The port sits at this board Z (between the two pegs keeps the heavy tracker low).
ATTACH_Z = -PEGBOARD.pitch / 2


@lru_cache(maxsize=1)
def _load_raw() -> Part:
    return import_step(STEP)


@lru_cache(maxsize=1)
def _posed() -> tuple[Part, tuple[float, float, float]]:
    """Return (posed_device, port_xyz) in posed-local coords (port at origin-ish).

    Posed-local = oriented apex -Y / strap +Y / port -Z, with the **port moved to
    the origin** so the connector/lean pivot is just the origin.
    """
    dev = _load_raw()
    dome = dev.solids()[DOME_SOLID]
    Dn = ocp_cloud(dome.wrapped, 0.3)
    cx, cy = float(Dn[:, 0].mean()), float(Dn[:, 1].mean())

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

    # Orient: native +Z (apex) -> -Y, then spin about Y so the port points -Z.
    orient = Rot(0, az + 90, 0) * Rot(90, 0, 0) * Pos(-cx, -cy, 0)
    port = (orient * Location((float(pc[0]), float(pc[1]), float(pc[2])))).position
    # Drop the port to the origin so it's the natural hang/lean pivot.
    to_origin = Pos(-port.X, -port.Y, -port.Z)
    posed = to_origin * orient * dev
    return posed, (0.0, 0.0, 0.0)


def _port_board_y() -> float:
    return BACKPLATE.thickness + TUNDRA.standoff


@lru_cache(maxsize=1)
def _angle() -> float:
    """The outward tilt: the param, or the auto-solved minimum clear angle."""
    if TUNDRA.mount_angle is not None:
        return TUNDRA.mount_angle
    posed, _ = _posed()
    pts = ocp_cloud(posed.wrapped, 0.5)
    return lean_angle(pts, (0.0, 0.0, 0.0), _port_board_y(), board_clear=TUNDRA.board_clearance)


def _place() -> Location:
    """Map posed-local (port at origin) into board coords: lean out, then position."""
    return Pos(0, _port_board_y(), ATTACH_Z) * lean_loc(_angle(), (0.0, 0.0, 0.0))


def seated_device() -> Part:
    """The tracker posed where it hangs on the magnet (for preview overlays)."""
    posed, _ = _posed()
    return _place() * posed


def _arm(clamp: Shape) -> Part:
    """The short horizontal arm of the 'L': bridges the clamp back to the plate face.

    The clamp hangs off this stub rather than being buried in a block, so nothing
    pushes back through the mounting plate. Sized to the clamp's outer width.
    """
    bb = clamp.bounding_box()
    y_back = BACKPLATE.thickness - 1.0
    y_front = bb.min.Y + 2.0
    return Pos(bb.center().X, (y_back + y_front) / 2, (bb.min.Z + bb.max.Z) / 2) * Box(
        clamp_outer_dia(), max(y_front - y_back, 1.0), bb.size.Z
    )


def dock() -> Part:
    """The Tundra dock: peg-back + a cable clamp on a short arm (an 'L'). One solid.

    The clamp grips the cable a body-length below the port and, leaned out, ends up
    standing ~5 mm off the board face — leaving room for the tracker's dome, which
    hangs toward the board. The entry slot faces +X so the part prints on its side
    (X up) without supports.
    """
    place = _place()
    clamp = place * connector_clamp(slot_dir=(1.0, 0.0))
    arm = _arm(clamp)
    return peg_back() + arm + clamp


def min_clear_angle() -> float:
    """Report the smallest outward tilt that clears the board (the auto-solve value).

    Handy for choosing a value to pin in ``Tundra.mount_angle``.
    """
    posed, _ = _posed()
    pts = ocp_cloud(posed.wrapped, 0.5)
    return lean_angle(pts, (0.0, 0.0, 0.0), _port_board_y(), board_clear=TUNDRA.board_clearance)
