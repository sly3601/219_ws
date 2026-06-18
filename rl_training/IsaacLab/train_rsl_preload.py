from pathlib import Path
import sys
import runpy

ROOT = Path(__file__).resolve().parent
SCRIPT_DIR = ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
TRAIN_PY = SCRIPT_DIR / "train.py"

sys.path.insert(0, str(SCRIPT_DIR))

# Windows GUI 模式下先预加载这些二进制模块，避免 Isaac Sim 启动后 DLL 冲突
import h5py
print("h5py preload ok")

import tensordict
print("tensordict preload ok")

from rsl_rl.runners import OnPolicyRunner
print("rsl_rl runners preload ok")

sys.argv = [str(TRAIN_PY)] + sys.argv[1:]

runpy.run_path(str(TRAIN_PY), run_name="__main__")
