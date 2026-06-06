# vrpegboard (WIP)

Q: Can Claude 4.8 Opus do CAD for me? 

A: sort of.

# Readme (slop) below

Parametric, 3D-printable charging mounts for VR gear that snap into a **1"
pegboard**. Each mount holds a device securely and presents a round magnetic
USB-C cable so the device charges as it seats.

Scope: 2× Valve Index controllers (L + R) and 3× Tundra trackers.

Built with [build123d](https://github.com/gumyr/build123d) (Python code-CAD).

## Quick start

```sh
uv sync --extra view --extra analyze  # build123d + OpenCascade; viewer; mesh/print analysis
uv run python fetch_models.py         # download vendor STEP CAD into vendor/
uv run vrpegboard                     # export STLs into out/ (+ overhang summary per part)
uv run pytest                         # manifold / bed-fit / peg-pitch checks
uv run python -m vrpegboard.fitcheck index        # geometric drop-in / cable press-in check
uv run python -m vrpegboard.printability out/*.stl # overhang + (if installed) PrusaSlicer stats
```

The Index controller booleans run in **mesh space (`manifold3d`)**, not OpenCascade — the
vendor controller is a genus-3, ~390k-triangle organic STEP that OCC can't boolean in
reasonable time (a swept-volume subtraction times out past minutes; manifold3d does it in
milliseconds). So `index_dock_*` come back as a watertight `trimesh`; everything else stays
build123d. `mesh.py` holds the manifold pipeline.

The **fit-check coupon** (`out/coupon.stl`) needs no vendor CAD — print it first
to validate board fit, the connector press-fit, and magnetic capture before
committing filament to full cradles.

### Seeing it against the wall

Two ways to eyeball the lean/standoff before printing:

- **Live 3D (recommended for tuning).** `ocp_vscode` (the `view` extra) ships a
  standalone **browser** viewer — no VS Code needed:

  ```sh
  uv run python -m ocp_vscode      # once: opens the viewer at localhost:3939
  uv run python scene.py index     # push Index dock + controller + pegboard panel
  uv run python scene.py tundra     # ...or the Tundra
  ```

  Tweak a knob (`MOUNT_ANGLE`/`STANDOFF`, or `Tundra.mount_angle`/`standoff`) and
  re-run `scene.py`; the viewer updates in place so you can rotate the assembly
  against the grey board panel and judge "will this look right against my wall".
  dock = blue, device = orange, board = grey.

- **Headless PNGs.** `uv run python preview.py` renders three orthographic
  scatter views (`out/preview_*.png`) with the same colour key — handy in a
  terminal-only session or for a quick diff.

## How the docks are generated

The magnetic cable is strong enough to carry these devices on its own (tested),
so the docks lean on it and stay minimal — they're built from the vendor STEP
geometry, not hand-modelled:

1. import the STEP and **auto-pose** it port-down (find the button "head" vs the
   handle "pommel/port", derive the handle axis, rotate vertical, spin the ring
   forward so it clears the board);
2. **angle the connector outward** so the device — hanging on the magnet — clears
   the board face instead of pressing into it. The tilt is *solved*: we tip the
   posed point cloud about the port until nothing is closer to the board than the
   clearance (`mount_angle = None` auto-solves; set a number to override);
3. for the **Index**, build a **holder block with the controller subtracted out of
   it** so the base nests into a pocket matching its own underside, drops in from the
   top, and rests on full contact — no upper ring. Crucially the subtracted shape is
   the controller **swept along its insertion axis** (a translational sweep), not the
   raw solid: the sweep backfills the controller's open concavities (the USB-C recess,
   the strap groove) so no block material floats inside the pocket, and opens the cavity
   straight along the axis so the controller drops in. The block is **axis-aligned to the
   board and reaches back to the board face**, hugs the handle (the ring sweeps out of
   it), and **drills out the modelled USB-C jack**. The **port frame** (where the cable
   clamp + jack sit) is **not guessed** — it's marked on the port face in FreeCAD and read
   in (see *Marking the port*), because the real port is off-centre and tilts ~20° off the
   handle axis. The **Tundra** gets **no cradle at all**, just the angled cable clamp;
4. hold the cable in a **c-channel clamp** (a short split tube) at the port. On the
   Tundra it rides a short horizontal **arm** off the peg-back — an "L", so nothing
   pushes back through the plate — and stands ~5 mm off the board, leaving room for
   the dome. Everything fuses into one printable solid.

The Index dock is assembled in mesh space and comes out a **watertight, single-body**
mesh (asserted in the tests). The Tundra dock and base parts are build123d solids; the
simple base parts are asserted OCC-valid.

## Marking the port

The vendor Index STEP models the USB-C port only as a recess and gives no axis — and the
real port is **off-centre and tilted ~20° off the handle axis**, so guessing it from the
handle-bottom centroid (the old approach) put the cable clamp ~9 mm off. Instead:

1. `uv run python -m vrpegboard.portmark` renders solid **depth views** of the handle
   underside (`out/port_*.png`) at a recorded mm scale, where the recess is visible.
2. In FreeCAD, draw a rectangle on the port face (a sketch coplanar with it) and **export
   it to both `.step` and `.3mf`**. The STEP carries the rectangle in the sketch's local
   plane; the `.3mf` `<build>` transform carries that plane's placement on the controller —
   together they recover the port's 3-D position + normal.
3. `portmark.port_frame_from_marked_step("vendor/...port_rect.step")` reads them, maps into
   the canonical pose, and prints `PORT_XY` / `PORT_Z` / `PORT_AXIS` to paste into
   `index_controller.py`.

## Checking fit before printing

`uv run python -m vrpegboard.fitcheck index` does two **geometric** (not physics) checks in
mesh space: it slides the real-size controller down its insertion axis and reports the
interference vs depth (a clear path = no undercut; snug seated contact is fine), the seated
clearance to the board, and whether the cable clamp's entry slot vents to air so the cable
can press in sideways. `printability.py` reports overhang area / a low-overhang orientation
and, if PrusaSlicer (flatpak) is installed, real slice time / filament / support stats.

## How the pegboard hook works

The top hook is **one circular profile swept along a smooth path**: straight
through the hole, then a filleted bend climbing up-and-back behind the board. To
mount, angle the whole part up so the hook lines up with the hole, slide it
through, then lower it to vertical — the hook swings behind the board and
**gravity locks** it (it can't pull out without lifting + tilting again). A lower
straight peg one pitch down stops rotation. The pegs are a loose slide fit on
purpose; the hook does the holding, not a tight peg. Because the bend is a single
smooth curve at ~45° (no flat overhang), the part prints tilted in the slicer
without support material. Tune the bend with `Pegboard.hook_bend_radius`.

## Charging cable retention

The cable is held by a **c-channel clamp** — a short split tube (like a flagpole
bracket) gripping the **thin 3 mm cable just below the rigid ~10 mm swivel body**,
not the 8 mm body itself. Gripping the cable keeps the clamp under ~7 mm across, so
it (and the whole Tundra dock) **prints on its side without supports**. The rigid
body rests its shoulder on the clamp's top rim and floats up to the port; the
magnet does the final centring, so the clamp only has to aim the cable and stop it
dropping out. You can't thread the cable through a closed bore, so it **snaps in
sideways** through a full-length slot (a touch under the cable, gripping past the
lips); the slot also vents the bore so it's never **blind**. `slot_dir` defaults
along **+X**, so laid on its side the slot points up off the bed.

## Status

- ✅ Coupon, peg-back (swept hook), c-channel cable clamp, **Index controller dock
  (L + R)**, **Tundra tracker dock** (×3, identical).
- The **Tundra** dock is magnet-only: peg-back + an outward-angled cable clamp on a
  short arm (an "L"), no framing. It's **≤7 mm thick so it prints on its side
  without supports**; the tracker hangs on the magnet, dome toward the board (with
  ~5 mm clearance to the clamp), strap plate and straps free. (~7 × 29 × 43 mm.)
- The **Index** dock is a **holder block with the controller (swept along its insertion
  axis) subtracted** — it nests into a pocket matching its underside and drops in from the
  top, no upper ring (magnet + gravity hold it, the pocket registers it). The port frame is
  marked in FreeCAD (off-centre, tilted ~20°), the jack is drilled through, and the cable
  clamp sits on the port axis. Assembled as a watertight mesh via manifold3d.
  (~35 × 68 × 47 mm.) `fitcheck index` confirms it drops in, clears the board, and the
  cable presses in.
- The outward tilt is **auto-solved** to just clear the board (with the real tilted port
  the Index now wants ~37° at `STANDOFF=15`; raise `STANDOFF` to trade lean for standoff —
  Tundra ~17°); `min_clear_angle()` in each device module reports the value, and
  `mount_angle`/`MOUNT_ANGLE` overrides it.

## Tuning

All dimensions live in `src/vrpegboard/params.py`. After a test print, adjust
there:

- `Pegboard.peg_clearance` (slide fit), `hook_angle` (climb angle / printability),
  `hook_bend_radius` (how tight the swept bend is), `catch_rise` / `catch_clearance`
  (the hook behind the board), `board_thickness`.
- `Connector.cable_od` / `cable_clearance` (cable bore grip), `cable_slot_width`
  (how hard the cable snaps in sideways), `clamp_wall` / `clamp_len` (clamp size —
  keep `cable_hole_dia + 2*clamp_wall` under the plate width so it prints on its
  side), `male_body_len` (sets the head height above the clamp).
- `Tundra.mount_angle` (`None` = auto-solve), `board_clearance`, `standoff` (tuned
  so the clamp stands ~5 mm off the board for the dome).

Per-device pose/fit knobs live atop the device modules:

- `index_controller.py` — `CUP_DEPTH` (how deep the pocket wraps the base — keep it
  in the straight handle region so the controller drops straight in), `CUP_CLEARANCE`,
  `CUP_WALL`, `JACK_DRILL_DIA`, and the mount geometry `MOUNT_ANGLE` (`None` =
  auto-solve), `SSTANDOFF` (trades against lean), `BOARD_CLEARANCE`.
- `tundra_tracker.py` — `ATTACH_Z` and the port-seed window; tilt via `Tundra`.

The pegboard plate/peg width (`Backplate.width`, `Pegboard.hole_dia`) is the max
feature that prints on its side support-free, so the cable clamp is sized to fit
under it — that's the whole reason it grips the thin cable rather than the body.

## Attribution / licenses

Vendor CAD is downloaded locally and **not redistributed** here (`vendor/` is
gitignored); check each source's terms before sharing derived geometry.

- Valve Index controller CAD — [ValveSoftware/IndexHardware](https://github.com/ValveSoftware/IndexHardware) (Creative Commons).
- Tundra Tracker developer model — [Tundra Labs docs](https://tundra-labs.github.io/tundra-tracker-docs/tracker_customization/).
- Pegboard hook geometry inspired by (not copied from)
  [pegstr](https://github.com/MGX3D/pegstr) and the
  [build123d pegboard hook](https://www.printables.com/model/343714-pegboard-hooks-build123d-customizable).

## Reference docks (for design ideas)

Existing hand-built docks worth studying before tuning — open the pictures/models
to see how they hSold the device, then translate into the band knobs above:

**Tundra trackers** (cradle very little; the straps/USB/magnet bear the weight):

- [Tundra Tracker Wall Charger — DevoCut (Printables)](https://www.printables.com/model/185755-tundra-tracker-wall-charger)
  — the plastic is "just a guide to find the USB port", charges with straps on;
  basically what our magnet-only Tundra dock does (this one is nice and thin).
- [Magnetic Charging Dock, Seven Bay, EOZ cables — Tsumitsuki (Thingiverse)](https://www.thingiverse.com/thing:5426361)
  — multi-bay, fits the official EOZ magnetic cable (same family as ours).
- [Magnetic Charging Dock, Seven Bay, NetDot Gen10 — Tsumitsuki (Thingiverse)](https://www.thingiverse.com/thing:5426357)

**Valve Index controllers** ("cup" the controller hanging port-down):

- [Index Controller charging holder — (Thingiverse 3728198)](https://www.thingiverse.com/thing:3728198)
  — the `box - controller` style our Index holder now uses: the controller's bottom
  is subtracted from a block so it nests in and rests on full contact, with a cable
  hole + lateral slot. Prints cleanly (only the pegboard pins want minor support).
- [Index Controller Charging Stand — jhawkn8r (Thingiverse 3339091)](https://www.thingiverse.com/thing:3339091)
  — the cleanest "cup around the base" rendition (holds the controller
  dead-vertical, which we angle out to clear the board).
- [Index Controller Magnetic Charging Stand — cups (Thingiverse)](https://www.thingiverse.com/thing:4797751)
- [Valve Index wall mount with controller charging dock (Printables)](https://www.printables.com/model/494201-valve-index-wall-mount-with-controller-charging-do)
- [valve index controller stand — STLFinder roundup](https://www.stlfinder.com/3dmodels/valve-index-controller-stand/)
