"""Turn slicing/overhang intuition into numbers an agent can read.

Two layers, both optional (under the ``analyze`` extra):

* **Geometry, in-process (trimesh).** ``overhang_report`` measures, for a given build
  direction, the down-facing area that slopes shallower than the support threshold (so it
  would need supports), the bed-contact area, and the bounding box — no slicer needed.
  ``best_orientation`` tries the six axis-aligned build directions and returns the one with
  the least overhang, a cheap proxy for "which way should this print?".
* **Ground truth (PrusaSlicer).** ``slice_report`` shells out to the installed PrusaSlicer
  (flatpak ``com.prusa3d.PrusaSlicer``) to actually slice an STL and parses the G-code
  header for print time / filament / **support** filament. Degrades gracefully if absent.

CLI: ``uv run python -m vrpegboard.printability out/*.stl``.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh

# Faces that face downward and slope shallower than this (degrees from horizontal) need
# support. ~45° is the classic FDM rule of thumb (PrusaSlicer's default is in this range).
SUPPORT_THRESHOLD_DEG = 45.0
FLATPAK_PRUSA = ["flatpak", "run", "com.prusa3d.PrusaSlicer"]


def _as_mesh(obj) -> trimesh.Trimesh:
    if isinstance(obj, trimesh.Trimesh):
        return obj
    m = trimesh.load(str(obj))
    return m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m


def overhang_report(
    obj,
    build_dir: tuple[float, float, float] = (0, 0, -1),
    threshold_deg: float = SUPPORT_THRESHOLD_DEG,
) -> dict:
    """Overhang / bed-contact metrics for a mesh printed along ``build_dir`` (gravity dir).

    ``build_dir`` points the way gravity does (down); layers stack opposite it. A face needs
    support if it faces *into* ``build_dir`` (down) and its slope from horizontal is below
    ``threshold_deg``. Returns areas in mm² plus the overhang fraction of total surface.
    """
    m = _as_mesh(obj)
    d = np.array(build_dir, float)
    d /= np.linalg.norm(d)
    n = m.face_normals
    areas = m.area_faces
    facing = n @ d  # >0 means the face points along build_dir (downward) = a candidate overhang
    slope = np.degrees(np.arccos(np.clip(facing, -1, 1)))  # 0 = points straight down (flat roof)
    overhang = (facing > 1e-3) & (slope < (90.0 - threshold_deg))
    # bed contact: faces ~flush with the lowest plane, facing straight down
    height = m.vertices @ d
    fl = m.triangles.mean(axis=1) @ d
    on_bed = (facing > 0.99) & (fl > height.max() - 0.3)
    total = float(areas.sum())
    lo, hi = m.bounds
    return {
        "overhang_area": float(areas[overhang].sum()),
        "overhang_fraction": float(areas[overhang].sum() / total) if total else 0.0,
        "overhang_faces": int(overhang.sum()),
        "bed_contact_area": float(areas[on_bed].sum()),
        "bbox": tuple(float(v) for v in (hi - lo)),
        "build_dir": tuple(float(v) for v in d),
    }


def best_orientation(obj, threshold_deg: float = SUPPORT_THRESHOLD_DEG) -> dict:
    """The axis-aligned build direction (of ±X/±Y/±Z) with the least overhang area."""
    m = _as_mesh(obj)
    dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    reports = [overhang_report(m, d, threshold_deg) for d in dirs]
    best = min(reports, key=lambda r: r["overhang_area"])
    return {"best": best, "all": reports}


def slice_report(
    stl: str | Path, config_ini: str | Path | None = None, supports: bool = True
) -> dict:
    """Slice an STL with PrusaSlicer (flatpak) and parse time/filament/support from the G-code.

    Returns ``{available: False, ...}`` if the flatpak (or a usable profile) isn't present,
    so callers can degrade gracefully rather than fail.
    """
    if shutil.which("flatpak") is None:
        return {"available": False, "reason": "flatpak not found"}
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "out.gcode"
        cmd = [*FLATPAK_PRUSA, "--export-gcode", "--output", str(out)]
        if supports:
            cmd += ["--support-material"]
        if config_ini:
            cmd += ["--load", str(config_ini)]
        cmd.append(str(stl))
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        except (subprocess.TimeoutExpired, OSError) as e:
            return {"available": False, "reason": str(e)}
        if not out.exists():
            # Without a config PrusaSlicer often refuses ("no loaded presets"); report why.
            return {
                "available": False,
                "reason": (p.stderr or p.stdout).strip()[:300] or "no g-code",
            }
        text = out.read_text(errors="ignore")

    def grab(pat: str) -> str | None:
        m = re.search(pat, text)
        return m.group(1).strip() if m else None

    return {
        "available": True,
        "print_time": grab(r"estimated printing time.*?=\s*(.+)"),
        "filament_mm": grab(r"filament used \[mm\]\s*=\s*([\d.]+)"),
        "filament_cm3": grab(r"filament used \[cm3\]\s*=\s*([\d.]+)"),
        "support_used": "support" in text.lower(),
    }


def summary_line(obj, name: str = "") -> str:
    """One-line overhang summary for the as-oriented mesh (used by the build export log)."""
    r = overhang_report(obj)
    b = best_orientation(obj)["best"]
    return (
        f"{name:20s} overhang {r['overhang_fraction'] * 100:4.1f}% "
        f"({r['overhang_area']:6.0f} mm^2)  best dir {tuple(int(v) for v in b['build_dir'])} "
        f"-> {b['overhang_fraction'] * 100:4.1f}%"
    )


def main(argv: list[str]) -> None:
    for path in argv:
        m = _as_mesh(path)
        print(summary_line(m, Path(path).name))
        sr = slice_report(path)
        if sr["available"]:
            print(
                f"    slice: time={sr['print_time']} filament={sr['filament_mm']}mm "
                f"support_in_gcode={sr['support_used']}"
            )
        else:
            print(f"    slice: unavailable ({sr['reason']})")


if __name__ == "__main__":
    main(sys.argv[1:])
