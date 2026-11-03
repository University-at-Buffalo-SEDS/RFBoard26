"""Prepare an actual host SDK for native C tests; never edit a source override."""
from __future__ import annotations
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import tomllib

MINIMUM = (4, 3, 6)
BUILD_FLAGS = ['build', '--crate-type', 'staticlib']


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(argv, *, cwd=None, env=None):
    result = subprocess.run(argv, cwd=cwd, env=env, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('Native SDK command failed: ' + ' '.join(map(str, argv)) +
                           '\n' + result.stdout + result.stderr)
    return result.stdout.strip()


def host_environment(original):
    env = dict(original)
    exact = {'RUSTFLAGS', 'RUSTC', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER',
             'RUSTC_BOOTSTRAP', 'CARGO_ENCODED_RUSTFLAGS', 'CC', 'CXX', 'AR',
             'RANLIB', 'CFLAGS', 'CXXFLAGS', 'CPPFLAGS', 'LDFLAGS', 'OUT_DIR',
             'DEVICE_IDENTIFIER', 'STARTING_QUEUE_SIZE', 'QUEUE_GROW_STEP',
             'PAYLOAD_COMPRESS_THRESHOLD'}
    prefixes = ('SEDSNET_', 'CARGO_TARGET_', 'CARGO_BUILD_', 'CARGO_PROFILE_',
                'MAX_', 'PKG_CONFIG_', 'CC_', 'CXX_', 'AR_', 'CFLAGS_')
    for key in list(env):
        if key in exact or key.startswith(prefixes):
            env.pop(key)
    env['CARGO_BUILD_JOBS'] = '2'
    env['CMAKE_BUILD_PARALLEL_LEVEL'] = '2'
    return env


def toolchain(source, env):
    rustc = run(['rustc', '-vV'], cwd=source, env=env)
    match = re.search(r'(?m)^host: (\S+)$', rustc)
    if not match:
        raise RuntimeError('rustc did not report a host target; install a host Rust toolchain.')
    return {'rustc': rustc, 'cargo': run(['cargo', '-V'], cwd=source, env=env),
            'cc': run(['cc', '--version'], env=env), 'host': match[1],
            'python': sys.version, 'platform': platform.platform()}



def cargo_configuration(root, source, env):
    candidates = set()
    for start in (root, source):
        for directory in (start, *start.parents):
            candidates.update(directory / '.cargo' / name for name in ('config', 'config.toml'))
    home = Path(env.get('CARGO_HOME', str(Path.home() / '.cargo')))
    candidates.update(home / name for name in ('config', 'config.toml'))
    return {str(path): sha(path) for path in sorted(candidates) if path.is_file()}


def native_archive_relative(host):
    filename = 'sedsnet.lib' if host.endswith('-msvc') else 'libsedsnet.a'
    return str(Path('target') / host / 'debug' / filename)

def source_files(source):
    top = subprocess.run(['git', '-C', str(source), 'rev-parse', '--show-toplevel'],
                         capture_output=True, text=True)
    if top.returncode == 0 and Path(top.stdout.strip()).resolve() == source.resolve():
        names = [p for p in subprocess.check_output(
            ['git', '-C', str(source), 'ls-files', '-z']).decode().split('\0') if p]
    else:
        excluded = {'.git', 'target', 'build', 'dist', '.venv', '__pycache__'}
        names = []
        for directory, dirs, files in os.walk(source):
            dirs[:] = [d for d in dirs if d not in excluded]
            names += [str((Path(directory) / f).relative_to(source)) for f in files]
    for config in ('Cargo.lock', '.cargo/config', '.cargo/config.toml', 'rust-toolchain', 'rust-toolchain.toml'):
        if (source / config).is_file():
            names.append(config)
    return {p: sha(source / p) for p in sorted(set(names)) if (source / p).is_file()}


def dependency_inputs(source):
    """Path dependencies can be outside a prepared SDK; include their inputs."""
    inputs = {}
    seen = {source.resolve()}
    pending = [source]
    while pending:
        current = pending.pop()
        manifest = tomllib.loads((current / 'Cargo.toml').read_text())
        def visit(value):
            if isinstance(value, dict):
                if isinstance(value.get('path'), str):
                    path = (current / value['path']).resolve()
                    if path not in seen and (path / 'Cargo.toml').is_file():
                        seen.add(path)
                        inputs[str(path)] = source_files(path)
                        pending.append(path)
                for child in value.values():
                    visit(child)
        for key in ('dependencies', 'build-dependencies', 'dev-dependencies', 'patch', 'target'):
            visit(manifest.get(key, {}))
    return inputs



def copy_inputs(source, files, external, stage):
    copied = stage / 'source'
    mappings = {source.resolve(): copied}
    snapshots = {source.resolve(): files}
    for dependency, inputs in external.items():
        dependency = Path(dependency).resolve()
        if dependency.is_relative_to(source.resolve()):
            continue  # Included in the root source snapshot.
        name = hashlib.sha256(json.dumps([str(dependency), inputs], sort_keys=True).encode()).hexdigest()[:32]
        mappings[dependency] = stage / 'dependencies' / name
        snapshots[dependency] = inputs
    for original, destination in mappings.items():
        destination.mkdir(parents=True, exist_ok=True)
        for name in snapshots[original]:
            relative = Path(name)
            if relative.is_absolute() or '..' in relative.parts:
                raise RuntimeError('Unsafe SDK input path: ' + name)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original / relative, target)
    rewrites = []
    for original, destination in mappings.items():
        for name in snapshots[original]:
            if Path(name).name != 'Cargo.toml':
                continue
            manifest = destination / name
            text = manifest.read_text()
            def relocate(match):
                value = tomllib.loads('path = ' + match[1])['path']
                dependency = (original / name).parent / value
                dependency = dependency.resolve()
                if not (dependency / 'Cargo.toml').is_file():
                    return match[0]  # [lib]/[[test]] paths name files, not crates.
                for prefix in sorted(mappings, key=lambda p: len(p.parts), reverse=True):
                    if dependency.is_relative_to(prefix):
                        target = mappings[prefix] / dependency.relative_to(prefix)
                        relative = Path(os.path.relpath(target, manifest.parent)).as_posix()
                        replacement = 'path = ' + json.dumps(relative)
                        if replacement != match[0]:
                            rewrites.append({'manifest': str(manifest.relative_to(stage)),
                                             'before': match[0], 'after': replacement})
                        return replacement
                raise RuntimeError('SDK path dependency was not snapshotted: ' + str(dependency))
            multiline = r"\bpath\s*=\s*(?:" + ('"' * 3) + "|" + ("'" * 3) + ")"
            if re.search(multiline, text):
                raise RuntimeError('Multiline dependency paths are unsupported; no SDK build was started.')
            changed = re.sub(r"\bpath\s*=\s*(\"(?:\\.|[^\"\\])*\"|'[^']*')", relocate, text)
            if changed != text:
                manifest.write_text(changed)
    visited = set()
    def verify(manifest):
        manifest = manifest.resolve()
        if manifest in visited:
            return
        visited.add(manifest)
        data = tomllib.loads(manifest.read_text())
        def visit(value):
            if isinstance(value, dict):
                if isinstance(value.get('path'), str):
                    target = (manifest.parent / value['path']).resolve()
                    if not target.is_relative_to(stage.resolve()) or not (target / 'Cargo.toml').is_file():
                        raise RuntimeError('Copied SDK dependency escapes the owned snapshot or is missing: ' + str(target))
                    verify(target / 'Cargo.toml')
                for child in value.values():
                    visit(child)
        for key in ('dependencies', 'build-dependencies', 'dev-dependencies', 'patch', 'target'):
            visit(data.get(key, {}))
    verify(copied / 'Cargo.toml')
    return copied, rewrites


def cached_dependencies(source, cache):
    return {str(Path(path).relative_to(cache)): files
            for path, files in dependency_inputs(source).items()}

def validate_source(source):
    if not all((source / p).is_file() for p in ('Cargo.toml', 'build.py', 'C-Headers/sedsnet.h')):
        raise RuntimeError('Native tests need a complete SEDSnet source (Cargo.toml, build.py, C-Headers/sedsnet.h).')
    version = tomllib.loads((source / 'Cargo.toml').read_text())['package']['version']
    match = re.match(r'^(\d+)\.(\d+)\.(\d+)', version)
    if not match or tuple(map(int, match.groups())) < MINIMUM:
        raise RuntimeError('Native cache tests require SEDSnet >=4.3.6; select current main or a compatible source override.')
    if 'seds_router_invalidate_network_variable_cache(' not in (source / 'C-Headers/sedsnet.h').read_text():
        raise RuntimeError('The selected SDK header lacks the 4.3.6 selective cache API.')


def select_source(root, cache, env):
    explicit = env.get('SEDSNET_TEST_SOURCE')
    if explicit:
        source = Path(explicit).resolve()
        validate_source(source)
        return source  # Never fetch, reset or prepare an explicit source.
    configured = {}
    fallbacks = []
    candidates = [root / 'build/Release/CMakeCache.txt', root / 'build/Debug/CMakeCache.txt',
                  *sorted((root / 'build').glob('*/CMakeCache.txt'))]
    for file in candidates:
        if not file.is_file():
            continue
        values = dict(re.findall(r'(?m)^([A-Za-z_][A-Za-z0-9_]*):[^=\r\n]*=([^\r\n]*)$', file.read_text()))
        if not configured:
            configured = values
        for key in ('SEDSNET_DIR', 'sedsnet_SOURCE_DIR', 'FETCHCONTENT_SOURCE_DIR_SEDSNET'):
            if values.get(key):
                fallbacks.append(Path(values[key]).resolve())
    selector = root / 'cmake/sedsnet_source.py'
    module = (root / 'cmake/sedsnet_source.cmake').read_text()
    match = re.search(r'set\(SEDSNET_REPOSITORY\s+"([^"]+)"', module)
    if not match or not selector.is_file():
        raise RuntimeError('The project SDK source selector is missing; restore cmake/sedsnet_source.py.')
    ref = configured.get('SEDSNET_GIT_REF', 'main')
    if ref not in ('main', 'dev'):
        raise RuntimeError('Configured SEDSNET_GIT_REF must be main or dev.')
    repository = configured.get('SEDSNET_REPOSITORY', match[1])
    argv = [sys.executable, str(selector), '--cache', str(cache / ('selected-' + ref)),
            '--ref', ref, '--repository', repository]
    for path in [*fallbacks, root / 'build/_deps/sedsnet-src', root / 'third_party/SEDSnet']:
        argv += ['--fallback', str(path)]
    truth = lambda value: str(value).lower() in ('1', 'true', 'on', 'yes')
    offline = any(truth(configured.get(key, '')) for key in
                  ('SEDSNET_OFFLINE', 'FETCHCONTENT_FULLY_DISCONNECTED',
                   'FETCHCONTENT_UPDATES_DISCONNECTED', 'FETCHCONTENT_UPDATES_DISCONNECTED_SEDSNET'))
    if offline or truth(env.get('CARGO_NET_OFFLINE', '')) or truth(env.get('SEDSNET_OFFLINE', '')):
        argv.append('--offline')
    source = Path(run(argv, cwd=root, env=env)).resolve()
    validate_source(source)
    return source


@contextmanager
def cache_lock(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a+b') as lock:
        if os.name == 'nt':
            import msvcrt
            if path.stat().st_size == 0:
                lock.write(b'0')
                lock.flush()
            lock.seek(0)
            msvcrt.locking(lock.fileno(), msvcrt.LK_LOCK, 1)
        else:
            import fcntl
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            if os.name == 'nt':
                lock.seek(0)
                msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def validate_override(source, archive, proof_path):
    if not proof_path:
        raise RuntimeError('SEDSNET_TEST_NATIVE_ARCHIVE needs matching SEDSNET_TEST_NATIVE_PROOF; omit both for automatic setup.')
    proof = json.loads(Path(proof_path).read_text())
    header = source / 'C-Headers/sedsnet.h'
    if proof.get('archive_sha256') != sha(archive) or proof.get('header_sha256') != sha(header):
        raise RuntimeError('The native SDK override archive/header does not match its proof.')
    if proof.get('source_inputs') is not None:
        if (proof['source_inputs'] != source_files(source)
                or proof.get('cached_dependencies', {}) != cached_dependencies(source, Path(proof_path).resolve().parent)):
            raise RuntimeError('The native SDK override source differs from its recorded inputs.')
    elif re.fullmatch(r'PUBLIC[0-9]+_PACKAGE_VERIFICATION_NATIVE_ARCHIVE', str(proof.get('state', ''))):
        if Path(proof['source']).resolve() != source.resolve():
            raise RuntimeError('The published native archive proof names a different SDK source.')
        head = run(['git', 'rev-parse', 'HEAD'], cwd=source)
        if head != proof.get('source_main') or run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=source):
            raise RuntimeError('Published SDK archive reuse requires the exact clean recorded source commit.')
        package = json.loads(Path(proof['package_proof']).read_text())
        version = tomllib.loads((source / 'Cargo.toml').read_text())['package']['version']
        if (package.get('version') != version or
                package.get('VCS', {}).get('git', {}).get('sha1') != head):
            raise RuntimeError('Published SDK archive proof version/source does not match the selected SDK.')
        lock = source / 'Cargo.lock'
        if not lock.is_file() or package.get('release_clone_lock_sha256') != sha(lock):
            raise RuntimeError('Published SDK archive reuse requires its exact recorded Cargo.lock, including ignored lock input.')

    else:
        raise RuntimeError('The native SDK override proof lacks source identity.')
    return {'source': str(source), 'header': str(header), 'archive': str(archive),
            'proof': str(Path(proof_path).resolve()), 'reused_verified_override': True}


def prepare_native_sdk(root, original=None):
    root = Path(root).resolve()
    original = dict(os.environ if original is None else original)
    cache = root / 'build/host-sdk'
    cache.mkdir(parents=True, exist_ok=True)
    # Serialize source selection too: two clean default invocations must not
    # race while the project's selector initializes its owned Git cache.
    with cache_lock(cache / 'prepare.lock'):
        source = select_source(root, cache, original)
        archive = original.get('SEDSNET_TEST_NATIVE_ARCHIVE')
        if archive:
            result = validate_override(source, Path(archive).resolve(), original.get('SEDSNET_TEST_NATIVE_PROOF'))
        else:
            env = host_environment(original)
            tools = toolchain(source, env)
            files = source_files(source)
            external = dependency_inputs(source)
            key_inputs = {'preparer_sha256': sha(Path(__file__)), 'files': files, 'path_dependencies': external, 'tools': tools,
                          'cargo_configuration': cargo_configuration(root, source, env),
                          'flags': BUILD_FLAGS + ['--target', tools['host']],
                          'header_sha256': sha(source / 'C-Headers/sedsnet.h')}
            key = hashlib.sha256(json.dumps(key_inputs, sort_keys=True).encode()).hexdigest()
            final = cache / key
            metadata = final / 'complete.json'
            if metadata.is_file():
                record = json.loads(metadata.read_text())
                actual = final / record['archive_relative']
                if (record['key_inputs'] != key_inputs or record['archive_sha256'] != sha(actual)
                        or record['source_inputs'] != source_files(final / 'source')
                        or record['header_sha256'] != sha(final / 'source/C-Headers/sedsnet.h')
                        or record['cached_dependencies'] != cached_dependencies(final / 'source', final)):
                    raise RuntimeError('Native SDK cache integrity failed; preserve the cache and inspect ' + str(metadata))
            else:
                stage = Path(tempfile.mkdtemp(prefix=key + '.building-', dir=cache))
                copied, rewrites = copy_inputs(source, files, external, stage)
                if toolchain(copied, env) != tools:
                    raise RuntimeError('Host toolchain differs after source isolation; retained snapshot: ' + str(stage))
                env['CARGO_TARGET_DIR'] = str(stage / 'target')
                command = [sys.executable, str(copied / 'build.py'), *BUILD_FLAGS, '--target', tools['host']]
                log = stage / 'build.log'
                with log.open('w') as output:
                    built = subprocess.run(command, cwd=copied, env=env, stdout=output, stderr=subprocess.STDOUT)
                relative = native_archive_relative(tools['host'])
                actual = stage / relative
                if built.returncode or not actual.is_file():
                    raise RuntimeError('Host SDK build failed; retained log: ' + str(log))
                if (source_files(source) != files or dependency_inputs(source) != external
                        or sha(Path(__file__)) != key_inputs['preparer_sha256']):
                    raise RuntimeError('SDK inputs changed during host build; retained evidence: ' + str(stage))
                record = {'key_inputs': key_inputs, 'source_inputs': source_files(copied),
                          'selected_source': str(source), 'archive_relative': relative,
                          'archive_sha256': sha(actual), 'header_sha256': sha(copied / 'C-Headers/sedsnet.h'),
                          'command': command, 'host_environment_isolated': True,
                          'owned_path_rewrites': rewrites,
                          'cached_dependencies': cached_dependencies(copied, stage)}
                (stage / 'complete.json').write_text(json.dumps(record, indent=2) + '\n')
                stage.replace(final)  # Only a complete, verified cache is published.
            result = {'source': str(final / 'source'), 'header': str(final / 'source/C-Headers/sedsnet.h'),
                      'archive': str(final / record['archive_relative']), 'proof': str(metadata),
                      'reused_verified_override': False}
    os.environ['SEDSNET_TEST_SOURCE'] = result['source']
    os.environ['SEDSNET_TEST_NATIVE_ARCHIVE'] = result['archive']
    os.environ['SEDSNET_TEST_NATIVE_PROOF'] = result['proof']
    return result


def configured_dependency_source(root, name, override_env):
    """Find a dependency selected by a real firmware configure, without writes.

    Explicit test overrides remain optional. A shared FetchContent base is as
    valid as build/<profile>/_deps; no private workspace path is assumed.
    """
    root = Path(root).resolve()
    explicit = os.environ.get(override_env)
    if explicit:
        candidate = Path(explicit).resolve()
        if not (candidate / 'CMakeLists.txt').is_file():
            raise RuntimeError('Invalid dependency source override: ' + str(candidate))
        return candidate
    candidates = []
    for cache in sorted(root.glob('build/*/CMakeCache.txt')):
        values = dict(re.findall(r'(?m)^([A-Za-z_][A-Za-z0-9_]*):[^=\r\n]*=([^\r\n]*)$', cache.read_text()))
        for key in ('FETCHCONTENT_SOURCE_DIR_' + name.upper(), name + '_SOURCE_DIR'):
            if values.get(key):
                candidates.append(Path(values[key]))
        if values.get('FETCHCONTENT_BASE_DIR'):
            candidates.append(Path(values['FETCHCONTENT_BASE_DIR']) / (name + '-src'))
        candidates.append(cache.parent / '_deps' / (name + '-src'))
    candidates.extend(root.glob('build/*/_deps/' + name + '-src'))
    for candidate in candidates:
        if not (candidate / 'CMakeLists.txt').is_file():
            continue
        if name == 'sedslaunchcore':
            header = candidate / 'bootloader/include/launchcore/persist.h'
            if not header.is_file() or 'launchcore_persist_get_strict' not in header.read_text():
                continue
        return candidate.resolve()
    raise RuntimeError('Configure the current firmware once to cache ' + name)
