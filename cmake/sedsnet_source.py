"""Select an online development branch or the last usable on-disk SEDSnet without editing it."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def git(*args, timeout=30):
    return subprocess.run(['git', *map(str, args)], check=True, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          timeout=timeout,
                          env={**os.environ, 'GIT_TERMINAL_PROMPT': '0'})


def usable(path):
    return all((path / f).is_file() for f in ('CMakeLists.txt', 'Cargo.toml', 'build.rs'))


def select_source(cache, repository, fallbacks, offline=False, timeout=15, ref="main"):
    if ref not in ("main", "dev"):
        raise ValueError("SEDSnet ref must be main or dev")
    cache.mkdir(parents=True, exist_ok=True)
    marker = cache / 'last-source.txt'
    if marker.exists():
        fallbacks = [Path(marker.read_text().strip()), *fallbacks]
    upstream = cache / 'upstream.git'
    if not offline:
        if not upstream.exists():
            git('init', '--bare', upstream)
        try:
            git('--git-dir', upstream, 'fetch', '--depth=1', '--no-tags', repository,
                f"+refs/heads/{ref}:refs/heads/{ref}", timeout=timeout)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            print(f'SEDSnet {ref} unavailable; using on-disk fallback ({type(error).__name__}).', file=sys.stderr)
        else:
            revision = git('--git-dir', upstream, 'rev-parse', f"refs/heads/{ref}").stdout.strip()
            destination = cache / revision
            if not destination.exists():
                # Build preparation modifies Cargo.toml/schema. Keep the upstream
                # object store pristine and never reset an existing working copy.
                with tempfile.TemporaryDirectory(prefix='checkout-', dir=cache) as tmp:
                    stage = Path(tmp) / 'source'
                    git('clone', '--shared', '--no-checkout', upstream, stage)
                    git('-C', stage, 'checkout', '--detach', revision)
                    if not usable(stage):
                        raise RuntimeError(f'Fetched {ref} is not a usable SEDSnet source tree')
                    stage.rename(destination)
            marker_tmp = marker.with_suffix('.tmp')
            marker_tmp.write_text(str(destination) + '\n')
            marker_tmp.replace(marker)
            print(f'SEDSnet {ref}: {revision}', file=sys.stderr)
            return destination
    for source in fallbacks:
        if usable(source):
            try:
                revision = git('-C', source, 'rev-parse', 'HEAD').stdout.strip()
            except subprocess.CalledProcessError:
                revision = 'vendored source (no Git revision)'
            print(f'SEDSnet offline/local: {source} [{revision}]', file=sys.stderr)
            return source
    raise RuntimeError('No usable SEDSnet source is available on disk. Connect once to fetch main, '
                       'or set FETCHCONTENT_SOURCE_DIR_SEDSNET to an existing source checkout.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--fallback', action='append', default=[], type=Path)
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--ref', choices=('main', 'dev'), default='main')
    args = parser.parse_args()
    try:
        print(select_source(args.cache.resolve(), args.repository, args.fallback, args.offline, ref=args.ref))
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
