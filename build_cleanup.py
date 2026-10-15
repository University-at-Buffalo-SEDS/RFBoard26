"""Remove board-owned build outputs and estimate the allocated space reclaimed."""
import os
from pathlib import Path
import shutil
import stat


def format_reclaimed(size: int) -> str:
    amount = float(size)
    for unit in ('B', 'KiB', 'MiB', 'GiB', 'TiB'):
        if amount < 1024 or unit == 'TiB':
            return f'{size} B' if unit == 'B' else f'{amount:.2f} {unit}'
        amount /= 1024


def clean_generated(build_root: Path, target: Path) -> tuple[list[Path], int]:
    build_root = Path(os.path.abspath(build_root))
    repo_root = build_root.parent
    target = Path(os.path.abspath(target))
    try:
        target.relative_to(build_root)
    except ValueError as error:
        raise ValueError(f'Refusing to clean outside the build directory: {target}') from error
    crc_root = repo_root / 'third_party' / 'embedded-crc32fast'
    candidates = [target, crc_root / 'target', crc_root / 'Cargo.lock']
    # Validate every ancestor before removing anything. Never follow a generated
    # directory (or a third_party parent) symlink into unrelated source/artifacts.
    for path in candidates:
        relative = path.relative_to(repo_root)
        current = repo_root
        for part in relative.parts:
            current /= part
            if current.is_symlink():
                raise ValueError(f'Refusing to clean through a symbolic link: {current}')
    present = [path for path in candidates if path.exists()]
    inodes = {}
    for path in present:
        files = (Path(folder) / name for folder, _, names in os.walk(path)
                 for name in names) if path.is_dir() else [path]
        for file in files:
            info = file.lstat()
            if not stat.S_ISREG(info.st_mode):
                continue
            key = (info.st_dev, info.st_ino)
            previous = inodes.get(key, (info, 0))
            inodes[key] = (info, previous[1] + 1)
    # Hard links only free storage when all links are removed. APFS clones and
    # snapshots can retain blocks, so the displayed total is an estimate.
    reclaimed = sum(getattr(info, 'st_blocks', None) * 512
                    if hasattr(info, 'st_blocks') else info.st_size
                    for info, count in inodes.values() if count >= info.st_nlink)
    for path in present:
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    return present, reclaimed
