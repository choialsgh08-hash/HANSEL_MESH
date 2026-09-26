from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]

def test_actual_firmware_loop_with_simulated_io(tmp_path):
    if not shutil.which("g++"):
        pytest.skip("C++ compiler unavailable")
    binary = tmp_path / "bridge-test"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "tests/firmware"),
                    str(ROOT / "tests/firmware/bridge_test.cpp"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=5)
