"""Assemble parts and export STLs into ``out/``.

Run with ``uv run vrpegboard`` (or ``uv run python -m vrpegboard.build``).

The *coupon* needs no vendor CAD and should be printed first to validate board
fit, the connector press-fit, and magnetic capture. Device docks require the
vendor STEP models (run ``fetch_models.py`` first) and are skipped if missing.
"""

from collections.abc import Callable
from pathlib import Path

from build123d import Part, Pos

from .connector import clamp_outer_dia, connector_clamp
from .mesh import bbox_size, export_solid
from .params import BACKPLATE, PEGBOARD
from .pegboard import peg_back

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "out"
VENDOR = ROOT / "vendor"


def coupon() -> Part:
    """Peg-back + a cable clamp on its face: the cheap first test print.

    Validates board fit, the clamp's press-fit/snap-in, and magnetic capture before
    committing filament to a full dock. The clamp overlaps the plate so they fuse
    (the real docks stand it off on an arm and lean it; here it just points up).
    """
    contact_y = BACKPLATE.thickness + clamp_outer_dia() / 2 - 1.0
    contact_z = -PEGBOARD.pitch / 2
    # Entry slot faces the room (+Y) so it's reachable on the coupon.
    clamp = Pos(0, contact_y, contact_z) * connector_clamp(slot_dir=(0.0, 1.0))
    return peg_back() + clamp


# A factory returns either a build123d Part (primitive docks) or a trimesh.Trimesh
# (the Index dock, assembled in mesh space); export_solid/bbox_size handle both.
Factory = Callable[[], object]

# Parts that never need vendor CAD.
BASE_PARTS: dict[str, Factory] = {
    "coupon": coupon,
    "pegback": peg_back,
    "connector_clamp": connector_clamp,
}


def _device_parts() -> dict[str, Factory]:
    """Device docks, included only when their vendor STEP is present."""
    parts: dict[str, Factory] = {}
    if (VENDOR / "index_controller.stp").exists():
        from .index_controller import dock_left, dock_right

        parts["index_dock_right"] = dock_right
        parts["index_dock_left"] = dock_left
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


if __name__ == "__main__":
    main()
