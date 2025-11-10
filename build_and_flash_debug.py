#!/usr/bin/env python3
from pathlib import Path
import subprocess
import importlib.util
import sys


def run(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def import_and_run_build():
    """Import build_debug.py and call its main() function."""
    spec = importlib.util.spec_from_file_location("build_debug", "build_debug.py")
    if spec is None or spec.loader is None:
        sys.exit("Error: could not load build_debug.py")

    build_debug = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build_debug)

    if hasattr(build_debug, "main"):
        print("→ Running build_debug.main()")
        build_debug.main()
    else:
        sys.exit("Error: build_debug.py has no main() function")


def main():
    # 1) Run build_debug.main()
    import_and_run_build()

    # 2) Flash binary
    bin_path = Path("build/Debug/PowerBoard26.bin")
    if not bin_path.exists():
        sys.exit(f"Error: binary not found at {bin_path}")

    run([
        "dfu-util",
        "-a", "0",
        "-s", "0x08000000",
        "-D", str(bin_path),
    ])


if __name__ == "__main__":
    main()
