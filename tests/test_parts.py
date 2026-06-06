"""Sanity checks: parts are valid single solids that fit the print bed."""

from pathlib import Path

import pytest

from vrpegboard.build import coupon
from vrpegboard.connector import connector_clamp
from vrpegboard.mesh import bbox_size
from vrpegboard.params import PEGBOARD, PRINT
from vrpegboard.pegboard import peg_back

VENDOR = Path(__file__).resolve().parents[1] / "vendor"
HAS_INDEX = (VENDOR / "index_controller.stp").exists()
HAS_TUNDRA = (VENDOR / "tundra_tracker.step").exists()


def _fits_bed(part) -> bool:
    # part is a build123d Part or a trimesh.Trimesh (the Index dock); bbox_size handles both.
    x, y, _ = bbox_size(part)
    bx, by = PRINT.bed
    return bx >= x and by >= y


@pytest.mark.parametrize("factory", [peg_back, connector_clamp, coupon])
def test_base_parts_are_valid_single_solids(factory):
    part = factory()
    assert part.is_valid
    assert len(part.solids()) == 1
    assert _fits_bed(part)


def test_connector_clamp_is_hollow():
    # The bore/recess/vent must remove material from the solid tube envelope.
    from math import pi

    from vrpegboard.connector import clamp_height, clamp_outer_dia

    solid_tube = pi * (clamp_outer_dia() / 2) ** 2 * clamp_height()
    assert connector_clamp().volume < solid_tube


def test_peg_back_spans_two_holes():
    # Backplate should be tall enough to cover the top hole and the lower peg.
    assert PEGBOARD.pitch < peg_back().bounding_box().size.Z


def test_peg_hook_reaches_behind_board():
    # The swept hook (retention catch) must reach past the board's rear face, else
    # it can't hook behind it. The deepest -Y geometry is the hook's tip.
    assert -PEGBOARD.board_thickness > peg_back().bounding_box().min.Y


def test_pegback_has_a_low_overhang_orientation():
    # Backs the "prints without supports" claim numerically (was only checked by width):
    # there should be a build direction where very little of the peg-back is steep overhang.
    from vrpegboard.mesh import part_to_trimesh
    from vrpegboard.printability import best_orientation

    best = best_orientation(part_to_trimesh(peg_back()))["best"]
    assert best["overhang_fraction"] < 0.15, best


def test_connector_cable_slot_breaches_side():
    # The lateral entry slot opens the bore to one face, so the cable can snap in
    # sideways and the bore never sits blind: removing the slot must add material
    # back (a closed tube has more volume than the slotted one).
    assert connector_clamp(slot=False).volume > connector_clamp(slot=True).volume


def test_cable_clamp_fits_sideways_print_width():
    # The clamp grips the thin cable specifically so it stays within the max feature
    # printable on its side (the plate/peg width); else it can't print support-free.
    from vrpegboard.connector import clamp_outer_dia
    from vrpegboard.params import BACKPLATE

    assert clamp_outer_dia() <= BACKPLATE.width + 0.1


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
def test_index_dock_is_single_solid_that_fits():
    from vrpegboard.index_controller import dock_left, dock_right

    # The Index dock is assembled in mesh space (manifold3d), so it comes back as a
    # watertight trimesh rather than a B-rep solid. What matters for printing is that
    # each dock is one connected piece (body) that fits the bed.
    for factory in (dock_right, dock_left):
        mesh = factory()
        assert mesh.body_count == 1, "dock must be one printable piece"
        assert mesh.is_watertight, "manifold-assembled dock must be watertight"
        assert _fits_bed(mesh)


@pytest.mark.skipif(not HAS_TUNDRA, reason="run fetch_models.py for vendor CAD")
def test_tundra_dock_is_single_solid_that_fits():
    from vrpegboard.tundra_tracker import dock

    part = dock()
    assert len(part.solids()) == 1, "dock must be one printable piece"
    assert _fits_bed(part)


@pytest.mark.skipif(not HAS_TUNDRA, reason="run fetch_models.py for vendor CAD")
def test_tundra_dock_prints_on_its_side():
    # Laid on its side (X up), every feature must fit the plate/peg width so it
    # prints without supports — the whole point of gripping the thin cable.
    from vrpegboard.params import BACKPLATE
    from vrpegboard.tundra_tracker import dock

    assert BACKPLATE.width + 0.5 >= dock().bounding_box().size.X


@pytest.mark.skipif(not HAS_TUNDRA, reason="run fetch_models.py for vendor CAD")
def test_tundra_tracker_clears_board():
    # The dock has no cradle; the angled connector must hold the hanging tracker
    # clear of the board face (Y=0), so the auto-solved lean has to actually work.
    from vrpegboard.tundra_tracker import seated_device

    assert seated_device().bounding_box().min.Y > 0.0


@pytest.mark.skipif(not HAS_INDEX, reason="run fetch_models.py for vendor CAD")
def test_index_controller_clears_board():
    # Same constraint for the controller: leaning out on its cup must keep every
    # part of it in front of the board face.
    from vrpegboard.index_controller import seated_device

    assert seated_device().bounding_box().min.Y > 0.0
