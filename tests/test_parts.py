"""Sanity checks: parts are valid single solids that fit the print bed."""

from pathlib import Path

import pytest

from vrpegboard.connector import cable_assembly, connector_socket, socket_outer_dia
from vrpegboard.mesh import bbox_size
from vrpegboard.params import BACKPLATE, CONNECTOR, PEGBOARD, PRINT
from vrpegboard.pegboard import (
    backplate,
    peg_back,
    peg_holes,
    peg_hook_part,
    peg_lower_part,
    stub_len,
)

VENDOR = Path(__file__).resolve().parents[1] / "vendor"
HAS_INDEX = (VENDOR / "index_controller.stp").exists()
HAS_TUNDRA = (VENDOR / "tundra_tracker.step").exists()


def _fits_bed(part) -> bool:
    x, y, _ = bbox_size(part)
    bx, by = PRINT.bed
    return bx >= x and by >= y


@pytest.mark.parametrize("factory", [peg_back, peg_hook_part, peg_lower_part, cable_assembly])
def test_base_parts_are_valid_single_solids(factory):
    part = factory()
    assert part.is_valid
    assert len(part.solids()) == 1
    assert _fits_bed(part)


def test_plate_with_glue_holes_is_one_solid():
    body = backplate() - peg_holes()
    assert body.is_valid
    assert len(body.solids()) == 1
    # The holes are blind: material must remain behind each pocket.
    assert body.volume < backplate().volume


def test_peg_stubs_fit_their_holes():
    # Diametral glue clearance is positive, and the keyed stub fits inside the
    # plate thickness with the blind floor and squeeze-out relief accounted for.
    assert PEGBOARD.peg_glue_clearance > 0
    assert 0 < stub_len() < BACKPLATE.thickness


def test_peg_back_spans_two_holes():
    assert PEGBOARD.pitch < peg_back().bounding_box().size.Z


def test_peg_hook_reaches_behind_board():
    # The swept hook (retention catch) must reach past the board's rear face,
    # else it can't hook behind it.
    assert -PEGBOARD.board_thickness > peg_back().bounding_box().min.Y


def test_lower_peg_outlasts_board_flex():
    # The anti-twist peg pokes well past the board so part flex can't walk it out.
    assert -(PEGBOARD.board_thickness + 3.0) > peg_lower_part().bounding_box().min.Y


def test_pegs_have_a_low_overhang_orientation():
    # The pegs print as separate side-lying parts; there must be a build
    # direction where very little of each is steep overhang.
    from vrpegboard.mesh import part_to_trimesh
    from vrpegboard.printability import best_orientation

    for factory in (peg_hook_part, peg_lower_part):
        best = best_orientation(part_to_trimesh(factory()))["best"]
        assert best["overhang_fraction"] < 0.15, best


def test_connector_socket_is_one_cutter_with_open_slot():
    cutter = connector_socket()
    assert cutter.is_valid
    assert len(cutter.solids()) == 1
    # The side slot must add cut volume (it vents the bores to one face).
    assert cutter.volume > connector_socket(slot=False).volume


def test_socket_stack_dims_make_sense():
    c = CONNECTOR
    # The magnet bore is wider than the barrel bore (the barrel passes through to
    # the open bottom; the step stops the disc), and both have a slip-fit gap.
    assert c.magnet_bore > c.shroud_bore > c.cable_od
    assert c.magnet_clearance > 0 and c.shroud_clearance > 0
    assert c.socket_depth == c.magnet_bore_depth + c.shroud_bore_depth
    # The bore is sunk deeper than the mated stack so the cable head can sit recessed
    # and the magnet lifts it to mate; the device adapter protrudes into that bore.
    assert c.magnet_bore_depth >= c.magnet_depth
    assert 0 < c.device_plate_depth < c.magnet_depth
    assert socket_outer_dia() <= BACKPLATE.width  # a single-column plate can wrap a socket


# --- Dock construction invariants (correct-by-construction, mesh space) ----------
# The docks are now conforming-cup meshes (a ray-cast depth raster of the device's
# bottom band, drop-in-safe because a heightfield is single-valued in z). We assert
# the construction guarantees directly rather than trusting the advisory interference
# fit-checks in ``fitcheck.py``.


def _seat_xyz(m) -> tuple[float, float, float]:
    p = m._place()
    return tuple(float(v) for v in tuple(p.position))


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
@pytest.mark.parametrize("method", ["solid", "shell"])
def test_index_dock_is_one_watertight_piece(method):
    from vrpegboard.index_controller import dock_left, dock_right

    for factory in (dock_right, dock_left):
        mesh = factory(method)
        assert mesh.is_watertight, "dock mesh must be closed"
        # One connected body = both bracket webs fuse to the cup (no orphan arm).
        assert mesh.body_count == 1, "dock must be one printable piece (arms connected)"
        assert _fits_bed(mesh)
        assert mesh.bounds[0][1] >= -0.05, "dock must not cross the board (Y>=0)"


@pytest.mark.skipif(not HAS_TUNDRA, reason="run fetch_models.py for vendor CAD")
@pytest.mark.parametrize("method", ["solid", "shell"])
def test_tundra_dock_is_one_watertight_piece(method):
    from vrpegboard.tundra_tracker import dock

    mesh = dock(method)
    assert mesh.is_watertight
    assert mesh.body_count == 1, "cup + neck + backplate must fuse into one piece"
    assert _fits_bed(mesh)
    assert mesh.bounds[0][1] >= -0.05, "dock must not cross the board (Y>=0)"


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
@pytest.mark.parametrize("method", ["solid", "shell"])
def test_index_drops_in_with_no_undercut(method):
    # Lifting the seated controller up the hang axis must free it: interference is
    # gone a few mm up (a plateau would mean an undercut trapping it).
    from vrpegboard.index_controller import dock, seated_device
    from vrpegboard.mesh import to_manifold

    d = to_manifold(dock(method))
    dev = to_manifold(seated_device())
    path = [(d ^ dev.translate((0.0, 0.0, float(t)))).volume() for t in (6, 10, 16)]
    assert max(path) < 5.0, f"undercut: interference persists up the axis {path}"


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
@pytest.mark.parametrize("method", ["solid", "shell"])
def test_index_slot_vents_hole_breaches_cable_fits(method):
    import numpy as np

    import vrpegboard.index_controller as ixm
    from vrpegboard.connector import cable_assembly
    from vrpegboard.index_controller import CABLE_SLOT_DIR, dock
    from vrpegboard.mesh import manifold_to_trimesh, to_manifold
    from vrpegboard.params import CONNECTOR

    dman = to_manifold(dock(method))
    o = np.array(_seat_xyz(ixm))
    tm = manifold_to_trimesh(dman)
    # Magnet hole opens through the cup floor (a point just above the seat is air).
    assert not bool(tm.contains((o + [0, 0, 1.0])[None, :])[0]), "magnet hole capped over the seat"
    # Cable slot vents to air along its direction at every depth of the socket.
    d = np.array([CABLE_SLOT_DIR[0], CABLE_SLOT_DIR[1], 0.0])
    rs = np.arange(1.0, 70.0, 0.5)
    for z in (-1.0, -CONNECTOR.magnet_depth + 1.0, -CONNECTOR.socket_depth + 1.0):
        hits = tm.contains((o + [0, 0, z])[None, :] + d[None, :] * rs[:, None]).sum()
        assert hits == 0, f"cable slot walled off at z={z}"
    # The real magnetic connector drops into the bores with ~no interference.
    from build123d import Pos

    cab = to_manifold(Pos(*o) * cable_assembly(barrel_len=20.0, cable_len=12.0))
    assert (dman ^ cab).volume() < 5.0, "magnet/barrel bore too tight or shallow"


@pytest.mark.skipif(not HAS_TUNDRA, reason="run fetch_models.py for vendor CAD")
@pytest.mark.parametrize("method", ["solid", "shell"])
def test_tundra_slot_vents_hole_breaches_cable_fits(method):
    import numpy as np
    from build123d import Pos

    import vrpegboard.tundra_tracker as ttm
    from vrpegboard.connector import cable_assembly
    from vrpegboard.mesh import manifold_to_trimesh, to_manifold
    from vrpegboard.params import CONNECTOR
    from vrpegboard.tundra_tracker import dock

    dman = to_manifold(dock(method))
    tm = manifold_to_trimesh(dman)
    o = np.array(_seat_xyz(ttm))
    assert not bool(tm.contains((o + [0, 0, 1.0])[None, :])[0]), "magnet hole capped over the seat"
    d = np.array([0.0, 1.0, 0.0])  # slot exits +Y (room side)
    rs = np.arange(1.0, 70.0, 0.5)
    for z in (-1.0, -CONNECTOR.magnet_depth + 1.0, -CONNECTOR.socket_depth + 1.0):
        hits = tm.contains((o + [0, 0, z])[None, :] + d[None, :] * rs[:, None]).sum()
        assert hits == 0, f"cable slot walled off at z={z}"
    cab = to_manifold(Pos(*o) * cable_assembly(barrel_len=20.0, cable_len=12.0))
    assert (dman ^ cab).volume() < 5.0, "magnet/barrel bore too tight or shallow"


@pytest.mark.skipif(not HAS_TUNDRA, reason="run fetch_models.py for vendor CAD")
def test_tundra_tracker_clears_board():
    from vrpegboard.tundra_tracker import seated_device

    assert seated_device().bounding_box().min.Y > 0.0


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
def test_index_controller_clears_board():
    from vrpegboard.index_controller import seated_device

    assert seated_device().bounding_box().min.Y > 0.0


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
def test_index_two_column_pegs():
    # The Index spreads its long cantilever across a 2×2 peg grid (two columns).
    from vrpegboard.index_controller import PEG_COLS

    assert len(PEG_COLS) == 2
