"""
tools/deploy.py — Upload BNO085 driver files to Pico.

Run from project root:
  python tools/deploy.py

Pico must be connected on COM7. mpremote interrupts any running script on connect.
"""

import subprocess
import sys
from pathlib import Path

PYTHON   = sys.executable
COM_PORT = "COM7"
ROOT     = Path(__file__).resolve().parent.parent

FILES = [
    ("src/bno08x.py", "bno08x.py"),
    ("src/i2c.py",    "i2c.py"),
]


def _upload(local_rel, remote_name):
    local = ROOT / local_rel
    if not local.exists():
        print(f"  MISSING  {local_rel}")
        return False
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "cp", str(local), f":{remote_name}"],
        capture_output=True, text=True, timeout=30,
    )
    if result.returncode != 0:
        print(f"  FAIL     {local_rel}: {result.stderr.strip()}")
        return False
    print(f"  OK       {local_rel} -> :{remote_name}")
    return True


def main():
    print(f"Deploying to Pico on {COM_PORT}...")
    ok     = sum(_upload(loc, rem) for loc, rem in FILES)
    failed = len(FILES) - ok

    print(f"\nDone: {ok} uploaded, {failed} failed.")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
