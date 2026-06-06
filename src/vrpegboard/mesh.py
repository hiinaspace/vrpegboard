"""Mesh-space geometry for the boolean-heavy device docks.

Why this exists: the Valve Index controller imports as a genus-3, ~390k-triangle
organic STEP. OCC/build123d booleans on it are pathologically slow — a single
*fused* drop-in sweep (union of the solid over its insertion travel) times out past
minutes, and even one multi-solid fuse never returns. ``manifold3d`` does the same
booleans in **milliseconds** and is robust to the high genus, so the device docks
that subtract/sweep the controller are assembled in mesh space and exported straight
to STL (which is a triangle mesh anyway). Simple primitives (peg-back, clamp, block)
are authored in build123d and tessellated here so they can join the fuse.

The one fiddly bit is getting a *valid* manifold out of a build123d STL export: the
per-B-rep-face triangulations leave hairline cracks at shared edges, and manifold3d
silently yields an **empty** solid (volume 0) if the mesh isn't watertight. Welding
vertices with an explicit ``digits_vertex`` (the default tolerance under-welds) plus
dropping duplicate faces fixes it; ``to_manifold`` asserts a non-empty result so a
regression is loud rather than silent.
"""

from __future__ import annotations

import os
import tempfile

import manifold3d as m3
import numpy as np
import trimesh
from build123d import Part, export_stl

# Linear deflection (mm) for tessellating B-rep parts before meshing. Fine enough
# that the contoured pocket keeps the controller's shape; coarse enough to stay light.
DEFAULT_TOL = 0.3
_MERGE_DIGITS = 4  # weld vertices to 1e-4 mm; the build123d default under-welds and
#                    leaves cracks that make manifold3d emit an empty (vol=0) solid.


def part_to_trimesh(part: Part, tol: float = DEFAULT_TOL) -> trimesh.Trimesh:
    """Tessellate a build123d ``Part`` to a welded, duplicate-free ``Trimesh``."""
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "part.stl")
        export_stl(part, p, tolerance=tol)
        tm = trimesh.load(p)
    assert isinstance(tm, trimesh.Trimesh), f"expected a single mesh from {p}, got {type(tm)}"
    tm.merge_vertices(digits_vertex=_MERGE_DIGITS)
    tm.update_faces(tm.unique_faces())
    tm.remove_unreferenced_vertices()
    return tm


def trimesh_to_manifold(tm: trimesh.Trimesh) -> m3.Manifold:
    """Wrap a (welded) ``Trimesh`` as a ``manifold3d.Manifold``; assert it's non-empty."""
    v = np.ascontiguousarray(tm.vertices, dtype=np.float32)
    f = np.ascontiguousarray(tm.faces, dtype=np.uint32)
    man = m3.Manifold(m3.Mesh(v, f))
    if man.volume() <= 1.0:
        raise ValueError(
            "manifold3d produced an empty solid — the input mesh isn't watertight "
            f"(faces={len(tm.faces)}, vol={man.volume():.3g}). Check the weld tolerance."
        )
    return man


def to_manifold(obj: Part | trimesh.Trimesh, tol: float = DEFAULT_TOL) -> m3.Manifold:
    """A ``manifold3d.Manifold`` from a build123d ``Part`` or a ``Trimesh``."""
    tm = obj if isinstance(obj, trimesh.Trimesh) else part_to_trimesh(obj, tol)
    return trimesh_to_manifold(tm)


def manifold_to_trimesh(man: m3.Manifold) -> trimesh.Trimesh:
    """Convert a ``Manifold`` back to a ``Trimesh`` (for export / preview / analysis)."""
    msh = man.to_mesh()
    verts = np.asarray(msh.vert_properties)[:, :3].copy()
    faces = np.asarray(msh.tri_verts).copy()
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)


def swept(
    man: m3.Manifold,
    direction: tuple[float, float, float],
    distance: float,
    n: int = 25,
) -> m3.Manifold:
    """The **translational swept volume** of ``man`` along ``direction`` over ``distance``.

    Approximated as the union of ``n`` copies translated evenly along the (unit-normalised)
    direction — the discretised version of sliding the solid through its travel. Two uses:

    * **constructively** — sweeping the controller up its insertion axis and subtracting
      that from a block makes a pocket that is open straight along the axis *and* has no
      leftover material in the controller's concavities (a recess at one height is cleared
      by solid passing through it at another), so the device drops in;
    * **as a feasibility check** — the same volume is what the device clears on its way in.

    manifold3d unions are sub-millisecond, so ``n`` can be generous; pick it so the step
    ``distance/(n-1)`` is well under the smallest concavity you need to clear.
    """
    d = np.asarray(direction, dtype=float)
    norm = float(np.linalg.norm(d))
    if norm < 1e-9:
        raise ValueError("direction must be nonzero")
    d = d / norm
    copies = [man.translate(tuple(d * (distance * i / (n - 1)))) for i in range(n)]
    return m3.Manifold.batch_boolean(copies, m3.OpType.Add)


def intersect_volume(a: m3.Manifold, b: m3.Manifold) -> float:
    """Volume (mm³) of the overlap of two manifolds — the interference for a fit check."""
    return (a ^ b).volume()


def bbox_size(obj: Part | trimesh.Trimesh) -> tuple[float, float, float]:
    """(X, Y, Z) extent of a build123d ``Part`` or a ``Trimesh``."""
    if isinstance(obj, trimesh.Trimesh):
        lo, hi = obj.bounds
        return tuple(float(x) for x in (hi - lo))
    s = obj.bounding_box().size
    return (s.X, s.Y, s.Z)


def bbox_min(obj: Part | trimesh.Trimesh) -> tuple[float, float, float]:
    """Lower (X, Y, Z) corner of the bounding box, for either representation."""
    if isinstance(obj, trimesh.Trimesh):
        return tuple(float(x) for x in obj.bounds[0])
    m = obj.bounding_box().min
    return (m.X, m.Y, m.Z)


def export_solid(obj: Part | trimesh.Trimesh, path: str) -> None:
    """Write an STL from either a build123d ``Part`` or a ``Trimesh``."""
    if isinstance(obj, trimesh.Trimesh):
        obj.export(path)
    else:
        export_stl(obj, path)
