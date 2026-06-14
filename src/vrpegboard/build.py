"""Assemble parts and export STLs into ``out/``.

Run with ``uv run vrpegboard`` (or ``uv run python -m vrpegboard.build``).

The pegs print as separate parts (lying on their side) and glue into the keyed
holes in each dock's back — print one ``peg_hook`` + one ``peg_lower`` per dock.
Device docks require the vendor STEP models (run ``fetch_models.py`` first) and
are skipped if missing. Print ``index_cup_test`` before the full Index docks: it
is the holder block alone, for validating the controller seat / magnet mate and
judging stability in hand.
"""

from collections.abc import Callable
from pathlib import Path

from build123d import Part

from .mesh import bbox_size, export_solid
from .pegboard import peg_hook_part, peg_lower_part

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "out"
VENDOR = ROOT / "vendor"

Factory = Callable[[], Part]

# Parts that never need vendor CAD. One hook + one lower peg per dock.
BASE_PARTS: dict[str, Factory] = {
    "peg_hook": peg_hook_part,
    "peg_lower": peg_lower_part,
}


def _device_parts() -> dict[str, Factory]:
    """Device docks, included only when their vendor STEP is present."""
    parts: dict[str, Factory] = {}
    if (VENDOR / "index_controller.stp").exists():
        from .index_controller import cup_test, dock_left, dock_right

        parts["index_dock_right"] = dock_right
        parts["index_dock_left"] = dock_left
        parts["index_cup_test"] = cup_test
    if (VENDOR / "tundra_tracker.step").exists():
        from .tundra_tracker import dock as tundra_dock

        # The three trackers are identical, so one STL is printed three times.
        parts["tundra_dock"] = tundra_dock
    return parts


def main() -> None:
    OUT.mkdir(exist_ok=True)
    parts = {**BASE_PARTS, **_device_parts()}
    for name, factory in parts.items():
        part = factory()
        path = OUT / f"{name}.stl"
        export_solid(part, str(path))
        x, y, z = bbox_size(part)
        print(f"{name:20s} -> {path.name:28s} ({x:6.1f} x {y:6.1f} x {z:6.1f} mm)")
        try:  # overhang summary is a nicety; never let the analyze extra break the build
            from .printability import summary_line

            print(f"  {summary_line(str(path))}")
        except Exception:  # noqa: BLE001
            pass
    print(
        "\nquantities of pegs: 2 hooks + 2 lower pegs per Index dock, "
        "1 + 1 per Tundra dock\n  (2 Index → 4 hooks + 4 lowers; 3 Tundra → 3 hooks + 3 lowers)"
    )


if __name__ == "__main__":
    main()
