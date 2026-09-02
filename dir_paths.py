import importlib.util
from pathlib import Path
import sys

_scripts_dir_paths = Path(__file__).resolve().parent / "scripts" / "dir_paths.py"
_spec = importlib.util.spec_from_file_location("tp_gmm_scripts_dir_paths", _scripts_dir_paths)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

for _k, _v in _mod.__dict__.items():
    if not _k.startswith("__"):
        globals()[_k] = _v