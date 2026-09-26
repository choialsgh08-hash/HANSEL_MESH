from pathlib import Path
import sys


REPOSITORY = Path(__file__).resolve().parents[1]
WORKSPACE = REPOSITORY / "ros2_ws"
SOURCE = WORKSPACE / "src"
for package_dir in SOURCE.iterdir():
    if package_dir.is_dir():
        sys.path.insert(0, str(package_dir))

