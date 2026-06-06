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
    lower_peg_extra: float = 1.0  # how far the anti-rotation peg pokes past the board

    @property
    def peg_dia(self) -> float:
        return self.hole_dia - self.peg_clearance

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

    # keep as thin as the pegs themselves so the plate can print
    # on its side without supports
    width: float = 7.0
    thickness: float = 5.0  # +Y depth, front of the board
    margin: float = 7.0  # plate extends this far above the top hole / below lower peg


@dataclass(frozen=True)
class Connector:
    """The aftermarket round magnetic USB-C cable (measured).

    Female plate stays in the device's port; the male end (cable side) press-fits
    into the print and presents its contact face upward toward the seated port.
    """

    female_plate_dia: float = 8.0  # measured
    female_protrude: float = 3.5  # measured — sticks out past the USB-C shell
    male_body_od: float = 8.0  # CONFIRM/tune — swivel body OD (press-fit target)
    male_body_len: float = 10.0  # measured — ~1 cm rigid swivel body
    cable_od: float = 3.0  # measured

    press_fit_interference: float = 0.2  # bore = body_od - this (diametral grip)
    cable_clearance: float = 0.6  # channel = cable_od + this
    plate_recess_clearance: float = 0.8  # lead-in recess around the female plate
    wall: float = 4.0  # material around the bore
    # You can't thread the cable through a closed bore (the USB plug end won't fit),
    # so the whole male end presses in **sideways** through a lateral slot shaped
    # like the cable's silhouette: wide for the swivel head, narrow for the cable.
    # Each width is a touch under the real part so it snaps past the slot lips and
    # is then captured by the round bore/channel behind them.
    head_slot_interference: float = 0.8  # head slot = body_bore_dia - this (snap-in lips)
    cable_slot_width: float = 2.4  # narrow neck the 3 mm cable snaps through
    # The clamp grips the thin 3 mm cable (not the 8 mm body), so it stays small
    # enough to print on its side within the peg/plate max feature (~7 mm). The
    # rigid body rests its shoulder on top of the clamp and floats up to the port;
    # the magnet does the centring. clamp_wall is sized so cable_hole_dia + 2*wall
    # stays under that width.
    clamp_wall: float = 1.6  # wall around the cable bore (cable_hole_dia + 2*this ≤ ~7 mm)
    clamp_len: float = 9.0  # how long the tube grips the cable

    @property
    def body_bore_dia(self) -> float:
        return self.male_body_od - self.press_fit_interference

    @property
    def head_slot_width(self) -> float:
        """Lateral entry width for the swivel head (under the bore so it snaps in)."""
        return self.body_bore_dia - self.head_slot_interference

    @property
    def cable_hole_dia(self) -> float:
        return self.cable_od + self.cable_clearance

    @property
    def plate_recess_dia(self) -> float:
        return self.female_plate_dia + self.plate_recess_clearance


@dataclass(frozen=True)
class Tundra:
    """Tundra tracker dock knobs.

    The magnetic cable alone is strong enough to hold a whole tracker (tested), so
    this dock has **no cradle** — just the peg-back and the magnetic connector,
    angled outward so the hanging tracker clears the board. The tracker registers
    on the magnet, port-down, dome toward the board, strap plate facing the room.
    """

    mount_angle: float | None = None  # outward cable tilt (deg); None = auto-solve
    board_clearance: float = 5.0  # min gap the hanging tracker keeps off the board face
    # The clamp grips the cable a body-length below the port and, leaned, ends up
    # nearer the board than the port; this standoff is tuned so the clamp body still
    # stands ~5 mm off the board face, leaving room for the dome.
    standoff: float = 9.0  # how far the port stands off the plate face (+Y)


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
