#!/usr/bin/env python3
import subprocess
from pathlib import Path
import sys


def run(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> None:
    project_dir = Path.cwd()
    build_dir = project_dir / "build" / "Debug"

    # Telemetry flag: ON by default, OFF if "no-telemetry" is passed
    telemetry_flag = "-DENABLE_TELEMETRY=ON"
    if "no-telemetry" in sys.argv[1:]:
        telemetry_flag = "-DENABLE_TELEMETRY=OFF"

    # 1) Configure with CMake
    run([
        "cmake",
        "-DCMAKE_BUILD_TYPE=Debug",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabihf.cmake"
        if False else "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake",
        "-DCMAKE_COMMAND=cmake",
        telemetry_flag,
        "-S", str(project_dir),
        "-B", str(build_dir),
        "-G", "Ninja",
    ])

    # 2) Build
    run([
        "cmake",
        "--build", str(build_dir),
        "--parallel",
    ])

    # 3) objcopy ELF → BIN
    elf_path = build_dir / "PowerBoard26.elf"
    bin_path = build_dir / "PowerBoard26.bin"

    run([
        "arm-none-eabi-objcopy",
        "-O", "binary",
        str(elf_path),
        str(bin_path),
    ])


if __name__ == "__main__":
    main()
