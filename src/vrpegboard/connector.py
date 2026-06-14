"""Socket geometry that captures the magnetic cable's two discs + barrel lead-in.

The aftermarket magnetic USB-C cable charges through **two magnetic discs**: a
USB-C adapter that stays plugged in the device, and the cable's magnetic head.
Mated, they stack to ``magnet_dia`` × ``magnet_depth`` and live in a bore
**below the device's port face** (which is the cup floor). Below the discs a
rigid swivel barrel (``shroud_dia``, ~20 mm long) runs to the flexible cable.

The dock seats the device on the cup floor (port face) and sinks the magnet stack
into a bore beneath it, so the swivel can't pivot the device over. It grips only a
short **lead-in** of the barrel and lets the rest of the barrel + cable hang free
out the open bottom — the first design tried to encapsulate the whole barrel and
came out far too shallow for the real magnet stack.

Local frame: **origin at the seat face** (= the device's port face = the cup
floor), +Z up toward the seated device. Going down (-Z):

* ``0 .. -magnet_bore_depth``: magnet bore (``magnet_bore``), holding both discs
  plus recess/lift travel; the step down to the barrel bore stops the cable's disc.
* ``-magnet_bore_depth .. -socket_depth``: barrel lead-in bore (``shroud_bore``).
* below: open — the barrel + cable hang free.

A cable-width **side slot** runs the full height out one face so you can route the
cable: drop the magnet head + barrel into the bores from the top (before the
device is placed), with the flexible cable laid out through the slot.

``connector_socket`` returns the **negative** (the cutter to subtract from a host
block); ``cable_assembly`` builds the cable for previews and fit checks.
"""

from math import atan2, degrees

from build123d import Box, Cylinder, Part, Pos, Rot

from .params import CONNECTOR


def socket_outer_dia() -> float:
    """Min outer diameter of material around the socket (widest bore + walls)."""
    return CONNECTOR.magnet_bore + 2 * CONNECTOR.socket_wall


def socket_bottom_z() -> float:
    """Z of the socket's working bottom (barrel lead-in end) in the local frame."""
    return -CONNECTOR.socket_depth


def device_plate() -> Part:
    """The device's magnetic USB-C adapter — the "female" plate that stays plugged
    into the device and protrudes ``device_plate_depth`` below the port face.

    Posed in the socket's local frame (origin at the port face, +Z toward the
    device), so it hangs from the seat down into the bore (``0 .. -device_plate_depth``)
    where the cable head lifts to meet it.
    """
    c = CONNECTOR
    return Pos(0, 0, -c.device_plate_depth / 2) * Cylinder(c.magnet_dia / 2, c.device_plate_depth)


def cable_assembly(barrel_len: float = 20.0, cable_len: float = 18.0) -> Part:
    """The magnetic cable end **in its mated position**, posed in the socket frame.

    Same frame as ``connector_socket``: **origin at the seat / port face**, +Z
    toward the seated device. The cable head is lifted so it couples with the
    ``device_plate`` (which protrudes ``device_plate_depth`` into the bore). From the
    mate face (``-device_plate_depth``) going down:

    * the cable's own magnetic disc (``magnet_dia``, ``magnet_depth - device_plate_depth``
      tall — the remainder of the mated stack), with a small USB-C male nub plugging
      back up into the device adapter;
    * then the rigid swivel barrel (``shroud_dia``, ``barrel_len``);
    * then the flexible ``cable_od`` cable.
    """
    c = CONNECTOR
    mate = -c.device_plate_depth  # the cable head couples with the device plate here
    plate_h = c.magnet_depth - c.device_plate_depth  # the cable's share of the mated stack
    plate = Pos(0, 0, mate - plate_h / 2) * Cylinder(c.magnet_dia / 2, plate_h)
    nub_h = min(3.0, c.device_plate_depth)  # USB-C male tip plugged up into the adapter
    nub = Pos(0, 0, mate + nub_h / 2) * Cylinder(2.2, nub_h)
    bz = mate - plate_h  # = -magnet_depth: top of the rigid barrel
    barrel = Pos(0, 0, bz - barrel_len / 2) * Cylinder(c.shroud_dia / 2, barrel_len)
    cz = bz - barrel_len
    cable = Pos(0, 0, cz - cable_len / 2) * Cylinder(c.cable_od / 2, cable_len)
    return plate + nub + barrel + cable


def connector_socket(
    slot_dir: tuple[float, float] = (1.0, 0.0),
    slot_reach: float = 25.0,
    slot: bool = True,
    slot_top: float | None = None,
) -> Part:
    """The socket **cutter** (subtract from a host block); seat face at origin.

    Stacked bores going down from the seat (z=0):

    * ``+1 .. -magnet_bore_depth``: magnet bore (``magnet_bore``) — poked 1 mm above
      the seat so it cleanly opens the cup floor; deeper than the mated stack so the
      cable head has recess/lift travel;
    * ``-magnet_bore_depth .. -socket_depth-2``: barrel lead-in bore (``shroud_bore``),
      run 2 mm past the working bottom so it vents the open underside.

    Plus a cable-width side slot along ``slot_dir`` (an (x, y) direction; default
    +X) reaching ``slot_reach`` from the axis, full height (from ``slot_top``,
    default just above the seat, down past the bottom) so the cable routes out.

    ``slot=False`` omits the slot — for sizing the host block around the socket
    (the walls must wrap everything *except* the slot, which must breach to air).
    """
    c = CONNECTOR
    z_mag = -c.magnet_bore_depth
    z_bot = socket_bottom_z()

    mag_h = c.magnet_bore_depth + 1.0  # +1: poke above the seat for a clean open cut
    magnet = Pos(0, 0, (1.0 + z_mag) / 2) * Cylinder(c.magnet_bore / 2, mag_h)
    bar_h = (z_mag - z_bot) + 2.0  # +2: poke past the working bottom to vent
    barrel = Pos(0, 0, z_mag - bar_h / 2 + 1.0) * Cylinder(c.shroud_bore / 2, bar_h)

    cutter = magnet + barrel
    if slot:
        z_top = 1.0 if slot_top is None else slot_top
        slot_h = z_top - (z_bot - 2.0)
        cut = Pos(slot_reach / 2 - 1.0, 0.0, z_top - slot_h / 2) * Box(
            slot_reach + 2.0, c.cable_slot_width, slot_h
        )
        angle = degrees(atan2(slot_dir[1], slot_dir[0]))  # +X is angle 0
        cutter += Rot(0, 0, angle) * cut
    return cutter
