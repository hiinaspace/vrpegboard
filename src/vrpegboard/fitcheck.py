"""Deterministic "does it assemble?" checks — geometry, not physics.

Both docks now hang the device **port straight down**, so the insertion axis is
board +Z (the device lifts straight up off its cup/cradle). Per device:

* **Drop-in** — slide the real-size device up its axis and report interference vs
  height. The cup/cradle is a cumulative silhouette (Index) / fitted V (Tundra),
  so a correct build shows ~0 interference all the way down; a plateau flags an
  undercut. Snug seated contact is fine — judge the path.
* **Cable fit** — the magnetic connector (two discs + barrel + cable) dropped onto
  the seat must sit in the bores with ~0 interference (the real "does the magnet
  end fit" test the first socket failed), and the side slot must vent to air so
  the cable can route out.
* **Socket walls** + **peg seats** — ring probes confirming full material around
  the magnet bore and around each glue pocket.

Run: ``uv run python -m vrpegboard.fitcheck index|tundra``.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .connector import cable_assembly
from .cradle import apply_location, ocp_cloud
from .mesh import manifold_to_trimesh, to_manifold
from .params import CONNECTOR, PEGBOARD


def _info(which: str) -> dict[str, Any]:
    if which == "index":
        from . import index_controller as m

        return {
            "dock": m.dock(),
            "device": m.seated_device(),
            "mate": m._place(),
            "slot": m.CABLE_SLOT_DIR,
            "cols": m.PEG_COLS,
            "mesh_device": True,  # the controller welds into a manifold
        }
    from . import tundra_tracker as m

    return {
        "dock": m.dock(),
        "device": m.seated_device(),
        "mate": m._place(),
        "slot": (0.0, 1.0),
        "cols": (0.0,),
        "mesh_device": False,  # the tracker STEP won't weld; use point containment
    }


def dropin(which: str, max_h: float = 28.0, step: float = 2.0) -> list[tuple[float, float]]:
    info = _info(which)
    dock_m = to_manifold(info["dock"])
    axis = np.array([0.0, 0.0, 1.0])  # lift straight up
    rows = []
    if info["mesh_device"]:
        dev_m = to_manifold(info["device"])
        for t in np.arange(0.0, max_h + 1e-6, step):
            rows.append((float(t), float((dock_m ^ dev_m.translate(tuple(axis * t))).volume())))
    else:
        tm = manifold_to_trimesh(dock_m)
        cloud = ocp_cloud(info["device"].wrapped, 0.3)
        for t in np.arange(0.0, max_h + 1e-6, step):
            rows.append((float(t), float(tm.contains(cloud + axis[None, :] * t).sum())))
    return rows


def cable_fit(which: str) -> float:
    """Interference (mm³) of the dropped-in magnetic connector against the dock."""
    info = _info(which)
    dock_m = to_manifold(info["dock"])
    cab = to_manifold(info["mate"] * cable_assembly(barrel_len=20.0, cable_len=12.0))
    return float((dock_m ^ cab).volume())


def slot_vent(which: str, max_reach: float = 45.0) -> dict[float, int]:
    """Rays from the socket axis out along the slot at several depths; 0 = vents."""
    info = _info(which)
    tm = manifold_to_trimesh(to_manifold(info["dock"]))
    mate, sd = info["mate"], info["slot"]
    out = {}
    for z in (-1.0, -CONNECTOR.magnet_depth + 1.0, -CONNECTOR.socket_depth + 1.0):
        o = apply_location(mate, np.array([[0.0, 0.0, z]]))[0]
        d = apply_location(mate, np.array([[sd[0], sd[1], z]]))[0] - o
        d /= np.linalg.norm(d)
        rs = np.arange(1.0, max_reach, 1.0)  # skip 0 (axis sits on the bore boundary)
        out[z] = int(tm.contains(o[None, :] + d[None, :] * rs[:, None]).sum())
    return out


def socket_walls(which: str) -> tuple[int, int]:
    info = _info(which)
    tm = manifold_to_trimesh(to_manifold(info["dock"]))
    mate, sd = info["mate"], info["slot"]
    az = np.radians(np.arange(0, 360, 20))
    slot_az = np.degrees(np.arctan2(sd[1], sd[0]))
    keep = np.abs(((np.degrees(az) - slot_az + 180) % 360) - 180) > 45  # off the slot
    r = CONNECTOR.magnet_bore / 2 + CONNECTOR.socket_wall / 2
    probes = []
    for z in (-2.0, -CONNECTOR.magnet_depth + 1.0):
        ring = np.column_stack([r * np.cos(az[keep]), r * np.sin(az[keep]), np.full(keep.sum(), z)])
        probes.append(apply_location(mate, ring))
    pts = np.vstack(probes)
    return int(tm.contains(pts).sum()), len(pts)


def peg_seats(which: str) -> tuple[int, int]:
    from .pegboard import GLUE_RELIEF, stub_len

    info = _info(which)
    tm = manifold_to_trimesh(to_manifold(info["dock"]))
    pb = PEGBOARD
    az = np.radians(np.arange(0, 360, 45))
    pts = []
    for cx in info["cols"]:
        for zc, dia in ((0.0, pb.peg_dia), (-pb.pitch, pb.lower_peg_dia)):
            r = (dia + pb.peg_glue_clearance) / 2 + 1.0
            for y in (1.5, 3.0):
                pts.append(
                    np.column_stack([cx + r * np.cos(az), np.full(len(az), y), zc + r * np.sin(az)])
                )
            pts.append(np.array([[cx, stub_len() + GLUE_RELIEF + 0.5, zc]]))  # blind floor
    p = np.vstack(pts)
    return int(tm.contains(p).sum()), len(p)


def report(which: str = "index") -> None:
    rows = dropin(which)
    unit = "mm^3" if _info(which)["mesh_device"] else "pts"
    print(f"{which} drop-in (height above seated -> interference, {unit}):")
    for t, v in rows:
        print(f"  {t:5.1f} mm  {v:9.1f}  " + "#" * int(min(v, 2000) / 40))
    path = [v for t, v in rows if t >= 4.0]
    limit = 100.0 if unit == "mm^3" else 5
    print(
        f"  path max (>=4 mm) = {max(path) if path else 0:.1f} {unit}  ->  "
        + ("no undercut, drops in" if (max(path) if path else 0) < limit else "UNDERCUT")
    )

    info = _info(which)
    clear = info["device"].bounding_box().min.Y
    print(
        f"board clearance (seated min Y) = {clear:.2f} mm  ->  {'clears' if clear > 0 else 'HITS'}"
    )

    cf = cable_fit(which)
    verdict = "magnet+barrel fit" if cf < 30 else "BORE TOO TIGHT/SHALLOW"
    print(f"cable fit interference = {cf:.1f} mm^3  ->  {verdict}")

    sv = slot_vent(which)
    ok = all(n == 0 for n in sv.values())
    print(f"cable slot vent (solid hits per depth) = {sv}  ->  {'vents' if ok else 'WALLED OFF'}")

    sw, swt = socket_walls(which)
    print(f"socket walls: {sw}/{swt} in solid  ->  {'intact' if sw == swt else 'THIN/MISSING'}")

    ps, pst = peg_seats(which)
    print(f"peg glue seats: {ps}/{pst} in solid  ->  {'intact' if ps == pst else 'PLATE CARVED'}")


if __name__ == "__main__":
    import sys

    report(sys.argv[1] if len(sys.argv) > 1 else "index")
