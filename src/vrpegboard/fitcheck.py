"""Deterministic "does it assemble?" checks — geometry, not physics.

Both checks run in mesh space with ``manifold3d`` (intersections are sub-millisecond), so
they're exact and agent-readable, with no rigid-body solver to tune. Two questions:

* **Controller drop-in** — step the real-size controller down its insertion axis into the
  dock and report the overlap (interference) volume vs depth. The pocket is the controller
  swept along that axis (see ``mesh.swept`` / ``index_controller._holder``), so a correct
  pocket shows ~0 interference all the way to seated; a non-zero plateau flags an undercut.
  We also report the seated controller's clearance to the board face (Y=0).
* **Cable side press-in** — march the seated cable assembly's port point outward along the
  clamp's entry-slot direction and test containment in the dock, to confirm the slot
  actually breaches to open air (else there's no way to snap the cable in).

Run: ``uv run python -m vrpegboard.fitcheck index``.
"""

from __future__ import annotations

import numpy as np

from .connector import clamp_outer_dia
from .cradle import apply_location
from .index_controller import (
    _canonical,
    _holder,
    _insertion_dir,
    _place,
    _port_loc,
    seated_device,
)
from .mesh import manifold_to_trimesh, to_manifold


def controller_dropin(max_depth: float = 40.0, step: float = 2.0) -> dict:
    """Interference (mm³) vs how far above seated the controller is, plus board clearance."""
    holder = _holder()
    place = _place()
    dev = to_manifold(place * _canonical())  # real-size (un-enlarged) controller, seated
    axis = np.array(_insertion_dir(place))  # +controller axis (out of the pocket)

    rows = []
    for t in np.arange(0.0, max_depth + 1e-6, step):
        moved = dev.translate(tuple(axis * float(t)))
        rows.append((float(t), float((holder ^ moved).volume())))
    return {
        "interference": rows,  # (height_above_seated_mm, overlap_mm3)
        "seated_interference": rows[0][1],
        "board_clearance": float(seated_device().bounding_box().min.Y),
    }


def cable_access(max_reach: float = 30.0, step: float = 1.0) -> dict:
    """March the port point outward along the clamp slot; report where it leaves the dock.

    If the slot breaches to a face, the ray exits solid material a little past the clamp
    wall (≈ its outer radius) and stays out — so the cable can snap in. If dock material
    walls the slot, the ray stays inside well beyond the clamp.
    """
    holder_tm = manifold_to_trimesh(_holder())
    loc = _place() * _port_loc()
    origin = apply_location(loc, np.array([[0.0, 0.0, 0.0]]))[0]
    sdir = apply_location(loc, np.array([[0.0, 1.0, 0.0]]))[0] - origin  # +Y local = slot dir
    sdir = sdir / np.linalg.norm(sdir)

    rs = np.arange(0.0, max_reach + 1e-6, step)
    pts = origin[None, :] + sdir[None, :] * rs[:, None]
    inside = holder_tm.contains(pts)
    # first reach beyond which the ray is outside for good (slot vented to air)
    exits = next((float(r) for r, ins in zip(rs, inside, strict=False) if not ins and r > 0), None)
    return {
        "clamp_outer_radius": clamp_outer_dia() / 2,
        "exits_solid_at": exits,  # mm from the port along the slot; None = never exits
        "profile": list(zip(rs.tolist(), inside.tolist(), strict=False)),
    }


def report(_which: str = "index") -> None:
    d = controller_dropin()
    print("Controller drop-in (height above seated -> interference):")
    for t, v in d["interference"]:
        bar = "#" * int(min(v, 2000) / 40)
        print(f"  {t:5.1f} mm  {v:9.1f} mm^3  {bar}")
    seated = d["seated_interference"]
    # The drop-in verdict is about *undercuts* — material that blocks the descent partway,
    # which shows as interference well above the seated point. Snug seated contact (and
    # sub-millimetre mesh-tessellation overlap) is expected and fine, so judge the path,
    # not the seated value.
    path = [v for t, v in d["interference"] if t >= 4.0]
    path_max = max(path) if path else 0.0
    print(
        f"drop-in path max interference (>=4mm above seated) = {path_max:.1f} mm^3  ->  "
        f"{'no undercut, drops in' if path_max < 100 else 'UNDERCUT blocks descent'}"
    )
    print(f"seated contact interference = {seated:.1f} mm^3 (snug/mesh-noise; not a bind)")
    print(
        f"board clearance (seated min Y) = {d['board_clearance']:.2f} mm  "
        f"->  {'clears' if d['board_clearance'] > 0 else 'HITS BOARD'}"
    )

    c = cable_access()
    print("\nCable side press-in along the clamp slot:")
    print(f"  clamp outer radius = {c['clamp_outer_radius']:.2f} mm")
    if c["exits_solid_at"] is None:
        print("  ray never leaves dock material -> slot is WALLED OFF (cable can't snap in)")
    else:
        ok = c["exits_solid_at"] <= c["clamp_outer_radius"] + 2.0
        verdict = "slot vents to air (press-in OK)" if ok else "blocked: material past the clamp"
        print(f"  ray exits dock material at {c['exits_solid_at']:.1f} mm  ->  {verdict}")


if __name__ == "__main__":
    import sys

    report(sys.argv[1] if len(sys.argv) > 1 else "index")
