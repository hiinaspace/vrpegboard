"""Render dock + seated-device overlays to PNGs for visual confirmation.

This is how you check the auto-pose and the connector/port alignment without a
3D viewer: the dock (printed part) is blue, the device sitting in it is orange.
The pegboard would be behind the part at -Y; the room is +Y.

    uv run python preview.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402

from scene import pegboard_panel  # noqa: E402
from vrpegboard.cradle import ocp_cloud  # noqa: E402

OUT = Path(__file__).resolve().parent / "out"
VIEWS = [(0, 1, "XY top"), (1, 2, "YZ side (board -Y / room +Y)"), (0, 2, "XZ front")]


def _cloud(obj) -> np.ndarray:
    """Point cloud (Nx3) from a build123d Part or a trimesh.Trimesh (the Index dock)."""
    if isinstance(obj, trimesh.Trimesh):
        return np.asarray(obj.vertices)
    return ocp_cloud(obj.wrapped)


def _overlay(dock, device, title, path):
    dp, gp = _cloud(dock), _cloud(device)
    bp = ocp_cloud(pegboard_panel(nx=3, nz=5).wrapped, 1.0)
    fig, axes = plt.subplots(1, 3, figsize=(15, 7))
    for ax, (i, j, lab) in zip(axes, VIEWS, strict=False):
        ax.scatter(bp[:, i], bp[:, j], s=0.4, alpha=0.15, color="tab:gray", label="board")
        ax.scatter(dp[:, i], dp[:, j], s=0.5, alpha=0.5, color="tab:blue", label="dock")
        ax.scatter(gp[:, i], gp[:, j], s=0.3, alpha=0.12, color="tab:orange", label="device")
        ax.set_aspect("equal")
        ax.set_title(lab)
        ax.set_xlabel("XYZ"[i])
        ax.set_ylabel("XYZ"[j])
        ax.grid(True, alpha=0.3)
        ax.legend(markerscale=6, fontsize=7)
    fig.suptitle(title)
    fig.savefig(path, dpi=80, bbox_inches="tight")
    plt.close(fig)
    print("->", path.relative_to(path.parents[1]))


def main() -> None:
    OUT.mkdir(exist_ok=True)
    from vrpegboard.index_controller import dock_right, seated_device

    _overlay(
        dock_right(), seated_device(), "Index dock + seated controller", OUT / "preview_index.png"
    )

    from vrpegboard.tundra_tracker import dock as tundra_dock
    from vrpegboard.tundra_tracker import seated_device as tundra_seated

    _overlay(
        tundra_dock(), tundra_seated(), "Tundra dock + seated tracker", OUT / "preview_tundra.png"
    )


if __name__ == "__main__":
    main()
