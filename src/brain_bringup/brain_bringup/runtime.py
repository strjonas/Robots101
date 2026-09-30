from __future__ import annotations

import os
import platform
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


def bringup_file(folder: str, file_name: str) -> str:
    """Path of a file in brain_bringup/<folder> (config/ or worlds/).

    Prefers the copy in the source tree (when ROBOTS101_ROOT is set, which
    scripts/ros_env.sh does) so that edits to the semantic map, the prompt, the
    RViz layout or the world apply on the next launch without rebuilding.
    """
    root = os.environ.get("ROBOTS101_ROOT", "").strip()
    if root:
        source_file = Path(root) / "src" / "brain_bringup" / folder / file_name
        if source_file.is_file():
            return str(source_file)
    return str(Path(get_package_share_directory("brain_bringup")) / folder / file_name)


def config_path(file_name: str) -> str:
    return bringup_file("config", file_name)


def world_path(file_name: str) -> str:
    return bringup_file("worlds", file_name)


def _link_short_plugin_dir(source_dir: Path, short_dir: Path) -> Path:
    short_dir.mkdir(parents=True, exist_ok=True)
    for plugin_path in source_dir.glob("*.dylib"):
        link_path = short_dir / plugin_path.name
        if link_path.is_symlink() or link_path.exists():
            if link_path.resolve() == plugin_path.resolve():
                continue
            link_path.unlink()
        link_path.symlink_to(plugin_path)
    return short_dir


def prepare_macos_gazebo_env() -> list[SetEnvironmentVariable]:
    if platform.system() != "Darwin":
        return []

    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if not conda_prefix:
        return []

    conda_lib = Path(conda_prefix) / "lib"
    ogre_dir = conda_lib / "OGRE"
    ogre2_dir = conda_lib / "OGRE-Next"
    if not ogre_dir.is_dir() or not ogre2_dir.is_dir():
        return []

    short_ogre_dir = _link_short_plugin_dir(ogre_dir, Path("/tmp/robots101-ogre"))
    short_ogre2_dir = _link_short_plugin_dir(ogre2_dir, Path("/tmp/robots101-ogre2"))

    dyld_library_path = os.environ.get("DYLD_LIBRARY_PATH", "")
    dyld_entries = [entry for entry in [str(conda_lib), dyld_library_path] if entry]
    dyld_value = ":".join(dyld_entries)

    # ros_gz_sim reads os.environ directly while building its ExecuteProcess.
    os.environ["DYLD_LIBRARY_PATH"] = dyld_value
    os.environ["OGRE_RESOURCE_PATH"] = str(short_ogre_dir)
    os.environ["OGRE2_RESOURCE_PATH"] = str(short_ogre2_dir)

    return [
        SetEnvironmentVariable("DYLD_LIBRARY_PATH", dyld_value),
        SetEnvironmentVariable("OGRE_RESOURCE_PATH", str(short_ogre_dir)),
        SetEnvironmentVariable("OGRE2_RESOURCE_PATH", str(short_ogre2_dir)),
    ]
