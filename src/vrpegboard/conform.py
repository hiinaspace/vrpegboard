"""Surface-conforming cups from a top-down depth raster of the device's bottom band.

The dock hangs the device **port straight down**; the cup wraps the bottom band of
the body for light registration against sideways torque. Earlier builds
approximated that band with a flat-floored silhouette prism (Index) or two fitted
planes (Tundra) — neither follows the real surface. Here we instead **ray-cast the
device mesh straight up** (a depth raster looking up the −Z axis): for each (x, y)
the first surface a vertical ray hits is the body's lower envelope ``D(x, y)``. A
heightfield is single-valued in z, so a cup built under it is **drop-in-safe by
construction** — the device can always descend straight onto it.

Two cup bodies are built from that raster, to compare in the viewer:

* ``"solid"`` — a conforming block: top surface = ``D − clearance`` (the device
  nests on it), flat bottom at ``floor_z``. The "box − controller" style.
* ``"shell"`` — a thin shell: top = ``D − clearance``, bottom offset inward along
  the surface normal by ``wall`` (slope-clamped). The literal "thin shell from the
  inner surface".

Both come back as ``manifold3d.Manifold`` so the device modules can fuse the
socket boss / bracket and subtract the connector socket in mesh space (the raster
is low-poly, so these booleans are cheap and robust — unlike a B-rep boolean
against the 390k-tri vendor mesh).
"""

from __future__ import annotations

import manifold3d as m3
import numpy as np
import trimesh

from .mesh import to_manifold


def device_field(
    mesh: trimesh.Trimesh, band_hi: float, res: float = 0.6, floor: float | None = None
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Ray-cast ``mesh`` straight up onto a grid; return ``(xs, ys, D, mask)``.

    ``D[i, j]`` is the z of the **first** surface a +Z ray hits at ``(xs[j], ys[i])``
    — the body's lower envelope (look up the −Z axis at the underside). ``mask`` is
    where that first hit lies in the cup band ``[floor, band_hi]`` (so the tracking
    ring high above the band, or gaps the ray passes clean through, drop out).
    """
    lo, hi = mesh.bounds
    z0 = float(lo[2]) - 1.0
    floor = float(lo[2]) - 1.0 if floor is None else floor
    xs = np.arange(lo[0] - res, hi[0] + 2 * res, res)
    ys = np.arange(lo[1] - res, hi[1] + 2 * res, res)
    gx, gy = np.meshgrid(xs, ys)  # (ny, nx)
    origins = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, z0)])
    dirs = np.tile((0.0, 0.0, 1.0), (origins.shape[0], 1))
    loc, idx_ray, _ = mesh.ray.intersects_location(origins, dirs, multiple_hits=False)

    D = np.full(origins.shape[0], np.inf)
    np.minimum.at(D, idx_ray, loc[:, 2])  # nearest hit per ray (defensive)
    D = D.reshape(gx.shape)
    mask = np.isfinite(D) & (band_hi >= D) & (floor <= D)
    return xs, ys, D, mask


def _largest_component(mask: np.ndarray) -> np.ndarray:
    """Keep only the dominant 4-connected blob (drops stray ring-arc specks)."""
    from scipy import ndimage

    lbl, n = ndimage.label(mask)
    if n <= 1:
        return mask
    sizes = ndimage.sum(mask, lbl, range(1, n + 1))
    return lbl == (1 + int(np.argmax(sizes)))


def _fill_holes(D: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Fill masked-but-missing interior cells from nearest valid neighbours."""
    from scipy import ndimage

    bad = mask & ~np.isfinite(D)
    if not bad.any():
        return D
    src = np.isfinite(D)
    idx = ndimage.distance_transform_edt(~src, return_distances=False, return_indices=True)
    out = D.copy()
    out[bad] = D[tuple(i[bad] for i in idx)]
    return out


def field_solid(
    xs: np.ndarray, ys: np.ndarray, ztop: np.ndarray, zbot: np.ndarray, mask: np.ndarray
) -> trimesh.Trimesh:
    """Watertight solid between two height-fields over the masked grid nodes.

    A quad is filled when all four corners are in ``mask``; the top gets that
    surface, the bottom its mirror, and every boundary edge (one that bounds a
    single filled quad) gets a vertical wall — so the result closes.
    """
    ny, nx = mask.shape
    gx, gy = np.meshgrid(xs, ys)
    idx = -np.ones((ny, nx), dtype=int)
    nv = int(mask.sum())
    idx[mask] = np.arange(nv)
    vt = np.column_stack([gx[mask], gy[mask], ztop[mask]])
    vb = np.column_stack([gx[mask], gy[mask], zbot[mask]])
    verts = np.vstack([vt, vb])

    q = mask[:-1, :-1] & mask[1:, :-1] & mask[:-1, 1:] & mask[1:, 1:]  # filled quads
    ii, jj = np.where(q)
    a = idx[ii, jj]
    b = idx[ii, jj + 1]
    c = idx[ii + 1, jj + 1]
    d = idx[ii + 1, jj]
    top = np.vstack([np.column_stack([a, b, c]), np.column_stack([a, c, d])])
    bot = np.vstack([np.column_stack([a, c, b]), np.column_stack([a, d, c])]) + nv

    e = np.sort(np.vstack([top[:, [0, 1]], top[:, [1, 2]], top[:, [2, 0]]]), axis=1)
    uniq, cnt = np.unique(e, axis=0, return_counts=True)
    bnd = uniq[cnt == 1]  # boundary edges of the filled region
    u, v = bnd[:, 0], bnd[:, 1]
    walls = np.vstack([np.column_stack([u, v, v + nv]), np.column_stack([u, v + nv, u + nv])])

    faces = np.vstack([top, bot, walls])
    tm = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    trimesh.repair.fix_normals(tm)
    return tm


def _socket_fill(mesh: trimesh.Trimesh, center, radius: float, top_z: float) -> trimesh.Trimesh:
    """Concatenate a solid plug at ``center`` (the port) into the raster mesh.

    The device's USB-C port is modelled **hollow**, so a vertical ray sinks into the
    recess and the cup floor would poke up into it (where the installed magnetic
    coupler actually sits). We plug it with a cylinder capped **flush with the
    surrounding bottom face** (the mesh's lowest z, less a hair) and wide enough to
    cover the recess opening — the raster then reads a flat face there instead of the
    cavity. The magnet bore removes the plug's centre anyway, so the cup just runs
    flush across the port instead of diving in.
    """
    z_lo = float(mesh.bounds[0][2]) - 0.5  # flush with (a hair below) the lowest face
    cyl = trimesh.creation.cylinder(
        radius=radius,
        height=top_z - z_lo,
        transform=trimesh.transformations.translation_matrix(
            (center[0], center[1], (z_lo + top_z) / 2)
        ),
    )
    return trimesh.util.concatenate([mesh, cyl])


def _surface(
    mesh: trimesh.Trimesh,
    band_hi: float,
    floor_z: float,
    clearance: float,
    res: float,
    max_r: float | None,
    center: tuple[float, float],
    fill: tuple[float, float] | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """``(xs, ys, surf, mask)``: the conforming **inner surface** (device lower
    envelope minus ``clearance``) the cup tops out at and the device rests on."""
    from scipy import ndimage

    if fill is not None:
        mesh = _socket_fill(mesh, center, fill[0], fill[1])
    xs, ys, D, mask = device_field(mesh, band_hi, res, floor=floor_z)
    if max_r is not None:
        gx, gy = np.meshgrid(xs, ys)
        mask &= np.hypot(gx - center[0], gy - center[1]) <= max_r
    mask = _largest_component(mask)
    D = _fill_holes(D, mask)
    # Conservative envelope: take the lowest device sample in a 3-cell window so the
    # discrete raster never sits *above* the true surface between grid nodes (which
    # would poke the cup into the device); clearance is then a true gap.
    valid = mask & np.isfinite(D)
    surf = np.where(valid, ndimage.minimum_filter(np.where(valid, D, np.inf), size=3), np.inf)
    return xs, ys, surf - clearance, mask  # inf outside the mask


def conforming_cup(
    mesh: trimesh.Trimesh,
    band_hi: float,
    floor_z: float,
    clearance: float,
    wall: float,
    method: str = "solid",
    res: float = 0.6,
    max_r: float | None = None,
    center: tuple[float, float] = (0.0, 0.0),
    fill: tuple[float, float] | None = None,
) -> m3.Manifold:
    """A conforming cup body (no socket bore yet) as a ``Manifold``.

    ``method="solid"`` nests the device on a conforming top over a flat
    ``floor_z`` bottom; ``method="shell"`` follows the surface with a ``wall``-thick
    skin (normal offset, slope-clamped). ``floor_z`` is the lowest cup material
    (where the socket boss / bracket pick it up). ``max_r`` keeps the cup compact
    around ``center`` (the port/grip), so a feature looping into the band far out
    (the Index tracking ring) is left to a clearance channel instead of cupped.
    ``fill=(radius, top_z)`` plugs the hollow port with the installed magnet coupler
    before rastering (see ``_socket_fill``).
    """
    from scipy import ndimage

    xs, ys, surf, mask = _surface(mesh, band_hi, floor_z, clearance, res, max_r, center, fill)
    top = np.where(mask, surf, floor_z)

    if method == "shell":
        # Offset inward along the surface normal, but take the gradient of a
        # **nearest-extended** surface (no cliff down to floor_z at the rim) so the
        # shell edge doesn't grow thin spikes plunging to the bottom.
        idx = ndimage.distance_transform_edt(
            ~np.isfinite(surf), return_distances=False, return_indices=True
        )
        ext = surf[tuple(idx)]
        dy, dx = np.gradient(ext, ys, xs)
        thick = wall * np.clip(np.sqrt(1.0 + dx * dx + dy * dy), 1.0, 2.0)
        bot = ext - thick
    else:  # solid
        bot = np.full_like(top, floor_z)

    bot = np.where(mask, np.minimum(bot, top - 0.4), floor_z)
    return to_manifold(field_solid(xs, ys, top, bot, mask))


def conforming_cavity(
    mesh: trimesh.Trimesh,
    band_hi: float,
    floor_z: float,
    clearance: float,
    res: float = 0.6,
    max_r: float | None = None,
    center: tuple[float, float] = (0.0, 0.0),
    fill: tuple[float, float] | None = None,
    ceiling: float = 60.0,
) -> m3.Manifold:
    """The drop-in cavity (region **above** the conforming surface, over the cup
    footprint) as a ``Manifold`` — subtract it from bracket arms so they don't fill
    the space the device occupies. Same args as ``conforming_cup`` so the cavity
    matches the cup it pairs with."""
    xs, ys, surf, mask = _surface(mesh, band_hi, floor_z, clearance, res, max_r, center, fill)
    cav_bot = np.where(mask & np.isfinite(surf), surf, floor_z)
    cav_top = np.where(mask, band_hi + ceiling, floor_z)
    return to_manifold(field_solid(xs, ys, cav_top, cav_bot, mask))
