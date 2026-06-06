"""C-channel clamp that holds the round magnetic cable by its thin cable.

The aftermarket magnetic USB-C cable carries the device on its own magnet, so the
print barely loads the connector — it only has to *aim* the cable at the seated
port and keep it from falling out. We grip the thin **3 mm cable** just below the
rigid ~10 mm swivel body (not the 8 mm body itself), so the clamp stays small
enough to print on its side within the peg/plate max feature (~7 mm). The body
rests its shoulder on top of the clamp and floats up to the port; the magnet does
the final centring.

Local frame: **origin at the device's port face** (where the magnetic head mates),
+Z up toward the seated device. Going down -Z:

* ``male_body_len`` of free space where the rigid swivel body sits (its shoulder
  rests on the clamp's top rim, which sets the head height),
* a short split tube gripping the cable, bored to the cable diameter,
* the bottom vents to open air so the cable exits.

You snap the cable in **sideways** through a full-length slot down one face (a
touch under the cable, so it grips past the lips); the slot also vents the bore so
it's never blind. Pick ``slot_dir`` so that face is open in the assembly — along
**+X** by default, so laid on its side for printing the slot points up off the bed
and the tube needs no support.
"""

from math import atan2, degrees

from build123d import Box, Cylinder, Part, Pos, Rot

from .params import CONNECTOR


def clamp_outer_dia() -> float:
    """Outer diameter of the cable clamp tube (cable bore + wall both sides)."""
    return CONNECTOR.cable_hole_dia + 2 * CONNECTOR.clamp_wall


def clamp_grip_top() -> float:
    """Z where the clamp tube begins (below the rigid body); the body rests here."""
    return -CONNECTOR.male_body_len


def clamp_height() -> float:
    """Length of the gripping tube."""
    return CONNECTOR.clamp_len


def cable_assembly(cable_len: float = 18.0) -> Part:
    """The aftermarket magnetic USB-C cable, posed in the clamp's local frame.

    Same frame as ``connector_clamp``: **origin at the port mate face**, +Z toward the
    seated device. Built to the measured ``Connector`` dims so the dock can be checked —
    visually (does the cable sit where the clamp/port expect?) and geometrically (can it
    press in sideways through the slot?). Going up/down from the mate face (z=0):

    * ``+Z``: the female plate (stays in the device port; protrudes ``female_protrude``);
    * ``0 .. -male_body_len``: the rigid swivel body whose shoulder rests on the clamp rim;
    * below that: the thin ``cable_od`` cable the clamp actually grips.
    """
    c = CONNECTOR
    plate = Pos(0, 0, c.female_protrude / 2) * Cylinder(c.female_plate_dia / 2, c.female_protrude)
    body = Pos(0, 0, -c.male_body_len / 2) * Cylinder(c.male_body_od / 2, c.male_body_len)
    cable_top = -c.male_body_len
    cable = Pos(0, 0, cable_top - cable_len / 2) * Cylinder(c.cable_od / 2, cable_len)
    return plate + body + cable


def connector_clamp(slot: bool = True, slot_dir: tuple[float, float] = (1.0, 0.0)) -> Part:
    """Return the cable clamp; port face at origin (+Z toward the device).

    A short split tube gripping the cable a body-length below the port, so the
    magnetic head presents at z=0. ``slot`` cuts the full-length lateral entry slot
    out to one face; ``slot_dir`` is the (x, y) direction it points — default
    ``(1, 0)`` (along +X) so it prints support-free on its side.
    """
    c = CONNECTOR
    outer_r = clamp_outer_dia() / 2
    z_top = clamp_grip_top()  # tube top (rigid body rests on this rim)
    z_bot = z_top - c.clamp_len
    h = z_top - z_bot
    mid = (z_top + z_bot) / 2

    tube = Pos(0, 0, mid) * Cylinder(outer_r, h)
    bore = Pos(0, 0, mid) * Cylinder(c.cable_hole_dia / 2, h + 2.0)  # through, vents both ends
    clamp = tube - bore

    if slot:
        # Full-length lateral entry slot, swept from the bore axis out through one
        # face (built along +X, then spun to slot_dir). A touch under the cable so
        # it snaps past the lips and the bore behind them holds it.
        reach = outer_r + 2.0
        slot_cut = Pos(reach / 2 - 1.0, 0.0, mid) * Box(reach, c.cable_slot_width, h + 2.0)
        angle = degrees(atan2(slot_dir[1], slot_dir[0]))  # +X is angle 0
        clamp = clamp - Rot(0, 0, angle) * slot_cut

    return clamp
