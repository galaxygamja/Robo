"""Developer-only mechanical export. Arduino IDE users need not run Python.

The checked-in .ino/src files are a complete, self-contained Arduino sketch.
Only copies public firmware source: never a secrets.h or local build product.
"""
from __future__ import annotations
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "arduino/RoboRobot/src"
HEADERS = ("robot_config.h", "robot_profiles.h", "lease_state.h", "json_envelope.h", "drive_output.h")


def expected_files():
    result = {name: (ROOT / "firmware/include" / name).read_text(encoding="utf-8").rstrip() + "\n"
              for name in HEADERS}
    firmware = (ROOT / "firmware/src/main.cpp").read_text(encoding="utf-8")
    for old, new in (("void setup()", "void roboSetup()"), ("void loop()", "void roboLoop()")):
        if firmware.count(old) != 1:
            raise ValueError(f"Expected exactly one {old}")
        firmware = firmware.replace(old, new)
    result["firmware.cpp"] = ('// Generated from firmware/src/main.cpp; edit the canonical source.\n'
                              '#include "../RobotSettings.h"\n' + firmware)
    result["firmware_entry.h"] = "#pragma once\nvoid roboSetup();\nvoid roboLoop();\n"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if checked-in sketch sources differ")
    args = parser.parse_args()
    files = expected_files()
    if args.check:
        actual = {p.name for p in DEST.iterdir() if p.is_file()}
        if actual != set(files):
            raise SystemExit("Arduino source list differs; run tools/sync_arduino_sketch.py")
        for name, content in files.items():
            if (DEST / name).read_text(encoding="utf-8") != content:
                raise SystemExit(f"Arduino export out of date: {name}")
        print("Arduino sketch source parity: PASS")
        return
    DEST.mkdir(parents=True, exist_ok=True)
    for name, content in files.items():
        (DEST / name).write_text(content, encoding="utf-8", newline="\n")
    print(f"Exported {len(files)} Arduino source files; no credentials copied")


if __name__ == "__main__":
    main()
