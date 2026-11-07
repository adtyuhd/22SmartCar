import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "host_side"), str(ROOT / "robot_side"), str(ROOT / "tools")]
