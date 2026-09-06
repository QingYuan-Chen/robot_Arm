"""Resolve declared package resources for source and installed simulation use."""

from pathlib import Path


def package_resource(package: str, relative: str) -> Path:
    source = Path(__file__).resolve().parents[2] / package / relative
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory

    installed = Path(get_package_share_directory(package)) / relative
    if not installed.exists():
        raise FileNotFoundError(installed)
    return installed
