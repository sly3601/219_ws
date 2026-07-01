from pathlib import Path
import sys
import runpy

ROOT = Path(__file__).resolve().parent
SCRIPT_DIR = ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
PLAY_PY = SCRIPT_DIR / "play.py"

sys.path.insert(0, str(SCRIPT_DIR))

import h5py
print("h5py preload ok")

import tensordict
print("tensordict preload ok")

from rsl_rl.runners import OnPolicyRunner
print("rsl_rl runners preload ok")

sys.argv = [str(PLAY_PY)] + sys.argv[1:]

runpy.run_path(str(PLAY_PY), run_name="__main__")
