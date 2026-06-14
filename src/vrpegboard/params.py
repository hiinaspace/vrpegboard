"""Central, tunable dimensions for all vrpegboard parts (millimetres, degrees).

Everything that might need adjusting after a test print lives here so there's a
single place to tune. Measured values are marked; the rest are sensible defaults.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Pegboard:
    """The pegboard this mount hooks into (measured).

    Retention works like a metal pegboard hook: the top peg goes through the hole
    and a tang behind the board turns **up** (+Z). You insert by angling the whole
    mount up so the tang lines up with the peg and slides through the hole, then
    lower it to vertical — the up-tang swings behind the board and gravity locks it
    there (it can't pull out without lifting + tilting again). The pegs are a loose
    slide fit (the tang does the holding, not a tight peg), so they go in easily.
    """

    pitch: float = 25.4  # 1" on-centre hole spacing (measured)
    hole_dia: float = 7.0  # measured — larger than US 1/4"
    board_thickness: float = 3.2  # 1/8" hardboard default; override if thicker

    # Peg / hook geometry. The peg is a deliberately loose slide fit; retention
    # comes from the hook prong behind the board, so don't make the peg snug.
    peg_clearance: float = 1.0  # diametral slop so pegs slide in easily (was snug)
    peg_lead_in: float = 1.5  # chamfered tip length so the peg self-starts into the hole
    catch_clearance: float = 0.6  # gap between the prong and the board's back face
    # The top peg goes straight through the hole, then a prong angles up-and-back
    # to hook behind the board. At 45° the bent pin is self-supporting, so it
    # prints (tilted ~45° in the slicer) without supports — unlike a 90° up-tang
    # whose underside is a flat overhang.
    hook_angle: float = 45.0  # prong angle above horizontal (45 = support-free)
    catch_rise: float = 8.0  # how far the prong rises (+Z) behind the board to lock
    hook_bend_radius: float = 2.5  # radius of the swept bend joining the straight run to the rise
    # The lower peg is the anti-twist feature: long enough that the part flexing
    # can't walk it out of its hole, and snugger than the hook peg (the hook stays
    # loose for easy insertion; yaw slop comes from the lower peg's clearance).
    lower_peg_extra: float = 5.0  # how far the anti-rotation peg pokes past the board
    lower_peg_clearance: float = 0.4  # snugger diametral fit than the hook peg

    # The pegs print as separate parts lying on their side (best layer orientation
    # for the bent hook) and glue into D-profile holes through the dock bodies, so
    # the bodies are free to print in whatever orientation suits their sockets.
    peg_glue_clearance: float = 0.25  # diametral gap stub-to-hole for the glue film
    peg_key_flat: float = 1.2  # depth of the D-flat that clocks the hook's tang upward

    @property
    def peg_dia(self) -> float:
        return self.hole_dia - self.peg_clearance

    @property
    def lower_peg_dia(self) -> float:
        return self.hole_dia - self.lower_peg_clearance

    @property
    def straight_peg_len(self) -> float:
        """Straight run of the top peg: through the board to just behind its rear."""
        return self.board_thickness + self.catch_clearance

    @property
    def lower_peg_len(self) -> float:
        return self.board_thickness + self.lower_peg_extra


@dataclass(frozen=True)
class Backplate:
    """Flat plate that bears on the board front; cradles fuse to its +Y face."""

    # The pegs glue in as separate parts now, so the plate no longer has to print
    # on its side under the ~7 mm max-feature limit — wide enough to bear on the
    # board and resist left/right twist.
    width: float = 16.0
    thickness: float = 5.0  # +Y depth, front of the board
    margin: float = 7.0  # plate extends this far above the top hole / below lower peg


@dataclass(frozen=True)
class Connector:
    """The aftermarket round magnetic USB-C cable (measured 2026-06-13).

    The charging interface is **two magnetic discs**: a USB-C adapter that stays
    plugged in the device's port + the cable's magnetic head. Mated, they stack to
    ``magnet_dia`` × ``magnet_depth`` and sit in a bore **below the device's port
    face** (the cup floor). Below the discs a rigid swivel barrel (``shroud_dia``,
    ~20 mm long) runs to the flexible ``cable_od`` cable.

    The device-side adapter protrudes ``device_plate_depth`` below the port face,
    so when the device is seated on the cup it pokes into the bore and the cable
    head lifts to meet it. The bore is sunk ``magnet_bore_depth`` (deeper than the
    mated stack) so the cable head can rest **recessed** below the seat when the
    device is off, then the magnet lifts it to couple while the cup still carries
    the body — instead of the body balancing on top of the cable.

    The dock does **not** encapsulate the whole barrel — it captures the magnet
    stack (so the swivel can't pivot the device over) plus a short lead-in of the
    barrel, and lets the rest of the barrel + cable hang free below the socket. A
    side slot lets the cable route out.
    """

    magnet_dia: float = 8.0  # measured — OD of the mated magnetic discs
    magnet_depth: float = 7.5  # measured — stacked height of both discs (mated)
    # The full-Ø magnet bore runs this deep before stepping down to the shroud bore.
    # Deeper than ``magnet_depth`` on purpose: the extra is recess/lift travel so the
    # cable head sits recessed when the device is off and the magnet lifts it to mate.
    magnet_bore_depth: float = 10.0
    device_plate_depth: float = 4.5  # measured — how far the device's magnetic adapter
    #                                  protrudes below the USB-C port face (into the bore)
    shroud_dia: float = 6.6  # measured — rigid swivel barrel / yoke OD
    shroud_bore_depth: float = 3.0  # how much of the barrel the socket grips (lead-in only)
    cable_od: float = 3.0  # measured — flexible cable below the barrel

    magnet_clearance: float = 0.4  # diametral slip fit for the magnet stack
    shroud_clearance: float = 0.4  # diametral slip fit for the barrel lead-in
    cable_slot_width: float = 3.4  # side slot the cable routes through (cable_od + slop)
    socket_wall: float = 2.0  # min material around any part of the socket bores

    @property
    def magnet_bore(self) -> float:
        return self.magnet_dia + self.magnet_clearance

    @property
    def shroud_bore(self) -> float:
        return self.shroud_dia + self.shroud_clearance

    @property
    def socket_depth(self) -> float:
        """Cutter depth below the seat (magnet bore + barrel lead-in bore)."""
        return self.magnet_bore_depth + self.shroud_bore_depth


@dataclass(frozen=True)
class Tundra:
    """Tundra tracker dock knobs.

    The magnetic cable carries the tracker's weight, but the swivel tip pivots, so
    the dock both **captures the rigid swivel body** (see ``Connector``) and wraps
    the tracker's bottom in a **shallow silhouette cup** so it can't tip. The
    tracker hangs port-down, dome toward the board, strap plate to the room,
    angled outward so it clears the board face.
    """

    mount_angle: float | None = None  # outward cable tilt (deg); None = auto-solve
    board_clearance: float = 5.0  # min gap the hanging tracker keeps off the board face
    # The socket block sits between the port and the board; this standoff keeps the
    # block (and the dome hanging toward the board) ~5 mm off the board face.
    standoff: float = 9.0  # how far the port stands off the plate face (+Y)
    cup_depth: float = 10.0  # how far the cup walls rise around the tracker's bottom
    cup_clearance: float = 0.6  # radial gap tracker-to-cup (drop-in slide)
    cup_wall: float = 2.5  # cup wall thickness around the silhouette pocket


@dataclass(frozen=True)
class Print:
    """Print/assembly tolerances and machine limits."""

    fit_clearance: float = 0.4  # general gap between print and a held device
    bed: tuple[float, float] = (220.0, 220.0)  # X, Y build area for the bed-fit test


PEGBOARD = Pegboard()
BACKPLATE = Backplate()
CONNECTOR = Connector()
TUNDRA = Tundra()
PRINT = Print()
