"""Parametric pegboard peg-back interface, shared by every device mount.

Coordinate convention (used by all parts in this project):

* Origin is the **centre of the top pegboard hole**, on the board's front face.
* +Y points away from the board (toward the room); the board occupies Y < 0.
* +Z is up, +X is to the right.

So pegs/hooks extend in -Y (into and through the board) and device cradles fuse
to the plate's +Y face. The back spans two holes: a top **hook** — one circular
profile swept along a smooth path that runs straight through the hole and then
curves up-and-back behind the board (insert angled-up, lower to lock — see
``params.Pegboard``) carrying the load — plus a lower straight peg for
anti-rotation. The bend is a single smooth curve (no flat overhang), so the part
prints tilted in the slicer without support material.
"""

from math import cos, radians, sin, tan

from build123d import (
    Axis,
    Box,
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Cone,
    Cylinder,
    Line,
    Part,
    Plane,
    Pos,
    Rot,
    fillet,
    sweep,
)

from .params import BACKPLATE, PEGBOARD


def _cyl_y(radius: float, length: float, at: tuple[float, float, float]) -> Part:
    """Cylinder whose axis runs along Y, centred at ``at``."""
    return Pos(*at) * Rot(90, 0, 0) * Cylinder(radius, length)


def _peg_y(radius: float, length: float, lead_in: float) -> Part:
    """A peg along -Y from the plate face (y=0) to y=-length, with a tapered tip.

    The tip tapers over ``lead_in`` so the peg self-starts into the hole.
    """
    body_len = length - lead_in
    body = _cyl_y(radius, body_len, at=(0, -body_len / 2, 0))
    # Cone built along +Z (base radius at -Z, top radius at +Z); Rot(90) sends +Z
    # to -Y, so the small end points toward the tip (-Y).
    tip = (
        Pos(0, -(body_len + lead_in / 2), 0) * Rot(90, 0, 0) * Cone(radius, radius * 0.55, lead_in)
    )
    return body + tip


def _hook(radius: float, straight: float, angle_deg: float, rise: float, bend_r: float) -> Part:
    """A single circular profile swept along a smooth hook path.

    The path runs from the plate face (origin) straight along -Y through the board
    for ``straight`` mm, fillets through ``bend_r``, then climbs up-and-back at
    ``angle_deg`` above horizontal until it has risen ``rise`` mm in +Z. One swept
    solid: no separate prong, no flat overhang, so it prints support-free.
    """
    a = radians(angle_deg)
    tail = rise / sin(a)  # path length of the climb to reach `rise` in +Z
    p0 = (0.0, 0.0, 0.0)
    p1 = (0.0, -straight, 0.0)  # corner, behind the board's rear face
    p2 = (0.0, p1[1] - cos(a) * tail, sin(a) * tail)
    with BuildPart() as bp:
        with BuildLine() as ln:
            Line(p0, p1)
            Line(p1, p2)
            fillet(ln.vertices().sort_by(Axis.Y)[1], radius=bend_r)  # round the corner
        with BuildSketch(Plane(origin=p0, z_dir=(0, -1, 0))):  # profile ⟂ start tangent
            Circle(radius)
        sweep()
    return bp.part


def peg_back() -> Part:
    """Return the pegboard back interface, positioned per the convention above.

    The plate's +Y face (at Y = ``BACKPLATE.thickness``) is where device cradles
    attach.
    """
    pb, bp = PEGBOARD, BACKPLATE
    r = pb.peg_dia / 2

    # Plate: front face on the board (Y=0), body extends +Y. Spans from `margin`
    # above the top hole to `margin` below the lower peg (one pitch down).
    z_top = bp.margin
    z_bottom = -pb.pitch - bp.margin
    plate_h = z_top - z_bottom
    plate = Pos(0, bp.thickness / 2, (z_top + z_bottom) / 2) * Box(bp.width, bp.thickness, plate_h)

    # Top hook: one swept profile, straight through the hole then curving up-and-back
    # behind the board; the mount's weight keeps it engaged (gravity lock). The
    # straight run extends past the bend's tangent point so the curve only begins
    # once clear of the board's rear face.
    setback = pb.hook_bend_radius * tan(radians(pb.hook_angle) / 2)  # fillet eats this much
    straight = pb.straight_peg_len + setback
    hook = _hook(r, straight, pb.hook_angle, pb.catch_rise, pb.hook_bend_radius)

    # Lower anti-rotation peg: a straight stub one pitch down (no hook needed).
    lower = _peg_y(r, pb.lower_peg_len, pb.peg_lead_in).moved(Pos(0, 0, -pb.pitch))

    return plate + hook + lower
