"""Live 3D scene for tuning the docks against a virtual pegboard wall.

This is the FreeCAD-like loop without leaving the code. ``ocp_vscode`` (the
``view`` extra) ships a **standalone browser viewer** — no VS Code needed:

    uv run python -m ocp_vscode          # once: opens the viewer at :3939
    uv run python scene.py index         # push the Index dock + controller + board
    uv run python scene.py tundra        # ...or the Tundra dock + tracker

Tweak a knob (``MOUNT_ANGLE``/``STANDOFF`` in ``index_controller.py``,
``Tundra.mount_angle``/``standoff`` in ``params.py``) and re-run ``scene.py``: the
viewer updates in place, so you can eyeball the lean and the standoff against the
grey pegboard panel — i.e. "will this look right against my wall".

    dock  = blue      device = orange (semi-transparent)      board = grey
"""

from __future__ import annotations

import sys

from build123d import Box, Cylinder, Part, Pos, Rot

from vrpegboard.params import PEGBOARD


def pegboard_panel(nx: int = 5, nz: int = 7) -> Part:
    """A chunk of 1" pegboard centred on the top hole (origin), holes on the grid.

    The board occupies Y in [-board_thickness, 0]; +Y is the room (matches the
    project convention). ``nx``/``nz`` are how many holes wide/tall to draw.
    """
    p = PEGBOARD
    w, h = nx * p.pitch, nz * p.pitch
    # Panel front face on Y=0, body into -Y. Shift down so the top hole row sits
    # near the top (the dock hangs from the top hole at the origin).
    z_top = p.pitch
    panel = Pos(0, -p.board_thickness / 2, z_top - h / 2) * Box(w, p.board_thickness, h)
    holes = None
    for ix in range(-(nx // 2), nx // 2 + 1):
        for iz in range(0, -nz, -1):
            hole = (
                Pos(ix * p.pitch, 0, z_top - p.pitch + iz * p.pitch)
                * Rot(90, 0, 0)
                * Cylinder(p.hole_dia / 2, p.board_thickness + 2)
            )
            holes = hole if holes is None else holes + hole
    return panel - holes if holes is not None else panel


def _scene(which: str):
    if which == "tundra":
        from vrpegboard.tundra_tracker import dock, min_clear_angle, seated_device

        return dock(), seated_device(), min_clear_angle()
    from vrpegboard.index_controller import dock_right, min_clear_angle, seated_device

    return dock_right(), seated_device(), min_clear_angle()


def main() -> None:
    which = sys.argv[1] if len(sys.argv) > 1 else "index"
    dock, device, angle = _scene(which)
    print(f"{which}: auto-solve lean = {angle}°  (override to tune)")

    try:
        from ocp_vscode import show
    except ImportError:
        sys.exit("ocp_vscode not installed — run: uv sync --extra view")

    try:
        show(
            pegboard_panel(),
            dock,
            device,
            names=["board", "dock", "device"],
            colors=["#9aa0a6", "#1f77b4", "#ff7f0e"],
            alphas=[0.45, 1.0, 0.35],
        )
    except Exception as e:  # viewer not running yet
        sys.exit(f"Could not reach the viewer ({e}).\nStart it first:  uv run python -m ocp_vscode")


if __name__ == "__main__":
    main()
