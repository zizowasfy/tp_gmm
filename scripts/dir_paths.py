#!/usr/bin/env python3

"""Shared directory helpers for the TP-GMM scripts.

The package is launched both from the source tree and from the ROS 2 install
tree. The task pickles must always live in the source workspace under
`ws_moveit/src/tp_gmm/tasks`, so this module resolves the workspace root first
and then anchors every path from that source package directory.
"""

from pathlib import Path
import sys


# def _find_workspace_root() -> Path:
#     current = Path(__file__).resolve()
#     for candidate in current.parents:
#         source_pkg = candidate / "src" / "tp_gmm"
#         if source_pkg.exists():
#             return candidate
#     raise FileNotFoundError("Could not locate ws_moveit workspace root")

# # WORKSPACE_ROOT = _find_workspace_root()

WORKSPACE_ROOT = Path('/home/zizo/the_folder/ws_moveit')
PKG_DIR = WORKSPACE_ROOT / "src" / "tp_gmm"

SCRIPTS_DIR = PKG_DIR / "scripts"
INCLUDE_DIR = PKG_DIR / "include"
DATA_DIR = PKG_DIR / "data"
TASKS_DIR = PKG_DIR / "tasks"

# The demonstrations live outside the package tree in the workspace root.
# EXTERNAL_ROOT = PKG_DIR.parent.parent.parent
EXTERNAL_ROOT = Path('/home/zizo/the_folder')
DEMONS_ROOT = EXTERNAL_ROOT / "Trajectory_Data_Collection" / "Demons"


def _ensure_sys_path(path: Path) -> None:
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.append(path_str)


def ensure_script_import_paths() -> None:
    _ensure_sys_path(INCLUDE_DIR)
    _ensure_sys_path(SCRIPTS_DIR)


def get_task_dir(task_name: str) -> Path:
    return TASKS_DIR / task_name


def get_task_file(task_name: str, filename: str) -> Path:
    return get_task_dir(task_name) / filename


def get_demonstrations_dir(task_name: str | None = None) -> Path:
    return DEMONS_ROOT / task_name if task_name else DEMONS_ROOT


def ensure_task_dir(task_name: str) -> Path:
    task_dir = get_task_dir(task_name)
    task_dir.mkdir(parents=True, exist_ok=True)
    return task_dir


def get_paths():
    return {
        "pkg_dir": str(PKG_DIR) + "/",
        "include_dir": str(INCLUDE_DIR) + "/",
        "scripts_dir": str(SCRIPTS_DIR) + "/",
        "data_dir": str(DATA_DIR) + "/",
        "tasks_dir": str(TASKS_DIR) + "/",
        "demons_dir": str(DEMONS_ROOT) + "/",
        "workspace_root": str(WORKSPACE_ROOT) + "/",
        "external_root": str(EXTERNAL_ROOT) + "/",
    }


ensure_script_import_paths()


if __name__ == "__main__":
    for name, path in get_paths().items():
        print(f"{name}: {path}")
