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
anti-rotation (snugger and longer than the hook so the part can't twist or walk
it out of its hole).

**The pegs are separate, glued parts.** The dock bodies print in whatever
orientation suits their pockets/sockets (the connector socket outgrew the old
"whole part ≤7 mm thick, printed on its side" trick), while the pegs print lying
on their side — the best layer orientation for the bent hook. Each peg has a stub
that glues into a hole through the dock's backplate; the hook's stub and hole are
D-profiled so it can only glue in with its tang pointing up.
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

GLUE_RELIEF = 0.3  # glue pocket past the stub's end so squeeze-out has somewhere to go
POCKET_FLOOR = 1.0  # material left behind a glue pocket (the holes are blind — the
#                     tundra plate has open air behind its top hole, and a blind
#                     floor gives the stub a depth stop everywhere)


def stub_len() -> float:
    """Glue stub length: fills the blind pocket up to the squeeze-out relief."""
    return BACKPLATE.thickness - POCKET_FLOOR - GLUE_RELIEF


def _cyl_y(radius: float, length: float, at: tuple[float, float, float]) -> Part:
    """Cylinder whose axis runs along Y, centred at ``at``."""
    return Pos(*at) * Rot(90, 0, 0) * Cylinder(radius, length)


def _hook(radius: float, straight: float, angle_deg: float, rise: float, bend_r: float) -> Part:
    """A single circular profile swept along a smooth hook path.

    The path runs from the plate face (origin) straight along -Y through the board
    for ``straight`` mm, fillets through ``bend_r``, then climbs up-and-back at
    ``angle_deg`` above horizontal until it has risen ``rise`` mm in +Z. One swept
    solid: no separate prong, no flat overhang, so it prints support-free lying on
    its side (the bend flat on the bed).
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


def _key_cut(r: float, y_lo: float, y_hi: float, flat_drop: float) -> Part:
    """Cutter for the clocking D-flat: everything above z = r - flat_drop, over y span."""
    h = 2 * r  # generous; only the slab above the flat plane matters
    return Pos(0, (y_lo + y_hi) / 2, (r - flat_drop) + h / 2) * Box(4 * r, y_hi - y_lo, h)


def peg_hook_part() -> Part:
    """The top hook peg, printed separately (lying on its side) and glued in.

    Glue stub spans y ∈ [0, stub_len] with the D-flat on top (+Z); the swept hook
    continues through the board and climbs behind it. The stub bottoms in its
    blind pocket, which sets the insertion depth.
    """
    pb = PEGBOARD
    r = pb.peg_dia / 2
    setback = pb.hook_bend_radius * tan(radians(pb.hook_angle) / 2)  # fillet eats this much
    straight = pb.straight_peg_len + setback
    hook = _hook(r, straight, pb.hook_angle, pb.catch_rise, pb.hook_bend_radius)
    stub = _cyl_y(r, stub_len(), at=(0, stub_len() / 2, 0))
    return (hook + stub) - _key_cut(r, -0.5, stub_len() + 0.5, pb.peg_key_flat)


def peg_lower_part() -> Part:
    """The lower anti-twist peg: straight, snug, with a tapered self-starting tip.

    Round (no clocking needed); the stub glues through the plate and the peg runs
    ``lower_peg_len`` past the board face so flex can't walk it out of its hole.
    """
    pb = PEGBOARD
    r = pb.lower_peg_dia / 2
    body_len = stub_len() + pb.lower_peg_len - pb.peg_lead_in
    body = _cyl_y(r, body_len, at=(0, stub_len() - body_len / 2, 0))
    tip_mid = stub_len() - body_len - pb.peg_lead_in / 2
    # Cone built along +Z (base radius at -Z, top radius at +Z); Rot(90) sends +Z
    # to -Y, so the small end points toward the tip (-Y).
    tip = Pos(0, tip_mid, 0) * Rot(90, 0, 0) * Cone(r, r * 0.55, pb.peg_lead_in)
    return body + tip


def peg_holes(cols: tuple[float, ...] = (0.0,)) -> Part:
    """Cutters for the glue holes — subtract from a dock body **after** fusing.

    One column per x-offset in ``cols`` (default a single central column; the
    Index uses two at ``x=±pitch/2``). Each column has a top hole at ``z=0``
    (D-profile matching the hook's keyed stub, flat up, so the tang can only point
    up) and a lower hole at ``z=-pitch`` (round). All are **blind** pockets
    (``stub_len + GLUE_RELIEF`` deep, ``POCKET_FLOOR`` left behind them), so the
    stubs bottom out with room for glue squeeze-out and the insertion depth is
    set even where nothing sits behind the plate.
    """
    pb = PEGBOARD
    pocket = stub_len() + GLUE_RELIEF
    depth = pocket + 1.0  # +1 starts the cut in front of the plate face
    g = pb.peg_glue_clearance
    r_top = (pb.peg_dia + g) / 2
    r_low = (pb.lower_peg_dia + g) / 2

    cols_cut = []
    for cx in cols:
        top = _cyl_y(r_top, depth, at=(cx, pocket - depth / 2, 0))
        # Hole flat sits half the glue clearance above the stub flat (uniform gap).
        top -= _key_cut(pb.peg_dia / 2, -1.0, pocket, pb.peg_key_flat - g / 2).moved(Pos(cx, 0, 0))
        low = _cyl_y(r_low, depth, at=(cx, pocket - depth / 2, -pb.pitch))
        cols_cut.append(top + low)
    out = cols_cut[0]
    for c in cols_cut[1:]:
        out = out + c
    return out


def placed_pegs(cols: tuple[float, ...] = (0.0,)) -> Part:
    """The glued-in pegs themselves (hooks + lower pegs), seated at each column.

    For previews/scene — the printed peg parts are ``peg_hook_part`` and
    ``peg_lower_part`` at the origin; here they're copied to each column offset
    (hook at z=0, lower peg a pitch down) and bundled into one ``Compound`` (the
    pegs are disjoint, so a plain fuse would degrade to a loose ``ShapeList``).
    """
    from build123d import Compound

    solids: list[Part] = []
    for cx in cols:
        solids.append(Pos(cx, 0, 0) * peg_hook_part())
        solids.append(Pos(cx, 0, -PEGBOARD.pitch) * peg_lower_part())
    return Compound(children=solids)  # ty: Compound is a Part subtype at runtime


def backplate(width: float | None = None) -> Part:
    """The bare mounting plate (no holes): front face on the board (Y=0), body +Y.

    Spans from ``margin`` above the top hole to ``margin`` below the lower peg.
    ``width`` defaults to the single-column ``Backplate.width``; pass a wider value
    for a multi-column grid (the Index spans two columns a pitch apart). Dock
    bodies fuse onto its +Y face, then subtract ``peg_holes()``.
    """
    bp, pb = BACKPLATE, PEGBOARD
    w = bp.width if width is None else width
    z_top = bp.margin
    z_bottom = -pb.pitch - bp.margin
    plate_h = z_top - z_bottom
    return Pos(0, bp.thickness / 2, (z_top + z_bottom) / 2) * Box(w, bp.thickness, plate_h)


def grid_width(cols: tuple[float, ...]) -> float:
    """Backplate width that covers a peg grid plus a margin each side."""
    span = max(cols) - min(cols)
    return span + PEGBOARD.peg_dia + 2 * BACKPLATE.margin


def peg_back() -> Part:
    """One-piece reference back (plate + glued-in pegs as a single solid).

    The printable parts are ``backplate() - peg_holes()`` plus the separate peg
    parts; this fused version exists for previews, tests, and geometry checks.
    """
    return backplate() + peg_hook_part() + peg_lower_part()
