"""Download vendor CAD (STEP) into ``vendor/``. Idempotent.

These files are NOT redistributed in this repo (license terms vary); this script
fetches them from the original sources on demand. See README for attribution.
"""

import io
import sys
import urllib.request
import zipfile
from pathlib import Path

VENDOR = Path(__file__).resolve().parent / "vendor"

# label -> (url, members to keep). Both upstreams ship zipped STEP.
SOURCES = {
    "index_controller": (
        "https://github.com/ValveSoftware/IndexHardware/raw/master/Controller/index_controller.stp.zip",
        None,  # keep all STEP-like members
    ),
    "tundra_tracker": (
        "https://github.com/tundra-labs/tundra-tracker-docs/raw/master/docs/files/"
        "TundraTracker_DeveloperModel%20v17.zip",
        None,
    ),
}

STEP_SUFFIXES = (".stp", ".step")


def _download(url: str) -> bytes:
    print(f"  downloading {url}")
    req = urllib.request.Request(url, headers={"User-Agent": "vrpegboard-fetch"})
    with urllib.request.urlopen(req) as resp:  # noqa: S310 (trusted vendor URLs)
        return resp.read()


def _extract_step(label: str, data: bytes, dest: Path) -> list[Path]:
    kept: list[Path] = []
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        for info in zf.infolist():
            name = info.filename
            if info.is_dir() or not name.lower().endswith(STEP_SUFFIXES):
                continue
            out = dest / f"{label}{Path(name).suffix.lower()}"
            out.write_bytes(zf.read(info))
            kept.append(out)
    return kept


def main() -> int:
    VENDOR.mkdir(exist_ok=True)
    for label, (url, _) in SOURCES.items():
        existing = [p for p in VENDOR.glob(f"{label}.*") if p.suffix.lower() in STEP_SUFFIXES]
        if existing:
            print(f"{label}: present ({existing[0].name})")
            continue
        print(f"{label}:")
        data = _download(url)
        kept = _extract_step(label, data, VENDOR)
        if not kept:
            print(f"  WARNING: no STEP file found in archive for {label}", file=sys.stderr)
        for p in kept:
            print(f"  -> {p.relative_to(VENDOR.parent)} ({p.stat().st_size // 1024} KiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
