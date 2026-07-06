"""Genesis Lite6 -- UFFactory Lite6 robot package for Genesis World."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path

__version__ = "0.1.0"

_ASSETS_DIR = Path(__file__).resolve().parent / "assets"
_URDF_DIR = _ASSETS_DIR / "urdf"
_CACHE_DIR = _URDF_DIR / ".cache"

_BASE_URDF = _URDF_DIR / "lite6_base.urdf"


def get_assets_dir() -> Path:
    """Return the absolute path to the assets directory."""
    return _ASSETS_DIR


def _resolve_mesh_paths(root: ET.Element, urdf_dir: Path) -> None:
    """Rewrite relative mesh filenames to absolute paths."""
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn and not os.path.isabs(fn):
            mesh.set("filename", str((urdf_dir / fn).resolve()))


def _merge_urdf(base_path: Path, eef_path: Path, out_path: Path) -> None:
    """Merge an EEF fragment URDF into the base arm URDF."""
    base_tree = ET.parse(str(base_path))
    eef_tree = ET.parse(str(eef_path))

    base_root = base_tree.getroot()
    eef_root = eef_tree.getroot()

    for child in eef_root:
        base_root.append(child)

    _resolve_mesh_paths(base_root, _URDF_DIR)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    base_tree.write(str(out_path), xml_declaration=True, encoding="unicode")


def _needs_rebuild(out_path: Path, *source_paths: Path) -> bool:
    """Check if the cached file is missing or older than any source."""
    if not out_path.exists():
        return True
    out_mtime = out_path.stat().st_mtime
    return any(s.stat().st_mtime > out_mtime for s in source_paths)


def get_urdf_path(eef: str | None = None) -> str:
    """Return the absolute path to a Lite6 URDF file.

    Args:
        eef: Name of the end-effector to attach. If None, returns the
            arm-only URDF. Supported values:
            - ``"ufgripper"`` -- UFactory Lite parallel gripper
            - Add more by placing ``eef_<name>.urdf`` in the urdf directory.

    Returns:
        Absolute path to the URDF file (ready for Genesis to load).
    """
    if not _BASE_URDF.exists():
        raise FileNotFoundError(f"Base URDF not found: {_BASE_URDF}")

    if eef is None:
        return str(_BASE_URDF)

    eef_path = _URDF_DIR / f"eef_{eef}.urdf"
    if not eef_path.exists():
        available = [
            f.stem.removeprefix("eef_")
            for f in _URDF_DIR.glob("eef_*.urdf")
        ]
        raise FileNotFoundError(
            f"EEF '{eef}' not found (no file eef_{eef}.urdf). "
            f"Available: {available}"
        )

    cached = _CACHE_DIR / f"lite6_{eef}.urdf"
    if _needs_rebuild(cached, _BASE_URDF, eef_path):
        _merge_urdf(_BASE_URDF, eef_path, cached)

    return str(cached)
