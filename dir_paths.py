#!/usr/bin/env python3

## System and directories stuff
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR / 'include'))
data_dir = str(ROOT_DIR / 'data/') + '/'
scripts_dir = str(ROOT_DIR / 'scripts') + '/'
# print(sys.path)