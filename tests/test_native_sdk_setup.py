"""SDK prerequisite orchestration tests; fake build output is never an SDK proof."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import threading
import hashlib
import time
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('native_sdk_for_tests', HERE / 'native_sdk.py')
sdk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sdk)
TOOLS = {'host': 'unit-host', 'rustc': 'unit rustc host: unit-host', 'cargo': 'unit cargo',
         'cc': 'unit cc', 'python': 'unit python', 'platform': 'unit platform'}
BUILD = r'''
import json,os,sys,time
from pathlib import Path
control=Path(os.environ['NATIVE_FIXTURE_CONTROL'])
with(control/'calls').open('a')as out:out.write('build\n')
(control/'started').touch()
if os.environ.get('NATIVE_FIXTURE_BARRIER'):
 deadline=time.monotonic()+5
 while not(control/'release').exists():
  if time.monotonic()>deadline:raise RuntimeError('fixture release missing')
  time.sleep(.001)
(control/'environment.json').write_text(json.dumps(dict(os.environ)))
(control/'argv.json').write_text(json.dumps(sys.argv))
if os.environ.get('NATIVE_FIXTURE_FAIL'):raise SystemExit(7)
host=sys.argv[sys.argv.index('--target')+1]
archive=Path(os.environ['CARGO_TARGET_DIR'])/host/'debug'/('sedsnet.lib'if host.endswith('-msvc')else'libsedsnet.a')
archive.parent.mkdir(parents=True,exist_ok=True)
archive.write_bytes(b'ORCHESTRATION UNIT MOCK, NOT A REAL SDK ARCHIVE')
'''

class NativeSDKSetupTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='native-sdk-setup-')
        self.path = Path(self.tmp.name)
        self.root = self.path / 'board'
        self.root.mkdir()
        self.source = self.path / 'sdk'
        (self.source / 'C-Headers').mkdir(parents=True)
        (self.source / 'Cargo.toml').write_text('[package]\nname="SEDSnet"\nversion="4.3.6"\n')
        (self.source / 'C-Headers/sedsnet.h').write_text('int seds_router_invalidate_network_variable_cache(void);\n')
        (self.source / 'build.py').write_text(BUILD)
        self.control = self.path / 'control'
        self.control.mkdir()
        self.env = dict(os.environ, SEDSNET_TEST_SOURCE=str(self.source),
                        NATIVE_FIXTURE_CONTROL=str(self.control))
        for name in ('SEDSNET_TEST_NATIVE_ARCHIVE', 'SEDSNET_TEST_NATIVE_PROOF'):
            self.env.pop(name, None)
        self.environ = patch.dict(os.environ)
        self.environ.start()
        self.tools = patch.object(sdk, 'toolchain', return_value=TOOLS)
        self.tools.start()

    def tearDown(self):
        self.tools.stop()
        self.environ.stop()
        self.tmp.cleanup()

    def prepare(self, **changes):
        return sdk.prepare_native_sdk(self.root, dict(self.env, **changes))

    def test_archive_name_matches_rust_host_abi(self):
        self.assertTrue(sdk.native_archive_relative('x86_64-pc-windows-msvc').endswith('sedsnet.lib'))
        self.assertTrue(sdk.native_archive_relative('aarch64-apple-darwin').endswith('libsedsnet.a'))
        self.assertTrue(sdk.native_archive_relative('x86_64-unknown-linux-gnu').endswith('libsedsnet.a'))

    def test_default_host_build_is_cached_and_source_is_unchanged(self):
        original = sdk.source_files(self.source)
        first = self.prepare()
        second = self.prepare()
        self.assertEqual(first['archive'], second['archive'])
        self.assertEqual((self.control / 'calls').read_text().splitlines(), ['build'])
        self.assertEqual(original, sdk.source_files(self.source))
        self.assertNotEqual(first['source'], str(self.source))
        self.assertTrue(Path(first['proof']).is_file())
        self.assertFalse(list((self.root / 'build/host-sdk').glob('*.building-*/complete.json')))

    def test_inherited_arm_and_application_environment_is_isolated(self):
        self.prepare(CARGO_BUILD_TARGET='thumbv7em-none-eabihf', CARGO_TARGET_DIR='/wrong',
                     CARGO_TARGET_THUMBV7EM_NONE_EABIHF_LINKER='arm-gcc',
                     RUSTFLAGS='--cfg embedded', CC='arm-none-eabi-gcc',
                     SEDSNET_EMBEDDED_SCHEMA_PATH='/wrong-schema', DEVICE_IDENTIFIER='AB')
        env = json.loads((self.control / 'environment.json').read_text())
        for key in ('CARGO_BUILD_TARGET', 'CARGO_TARGET_THUMBV7EM_NONE_EABIHF_LINKER',
                    'RUSTFLAGS', 'CC', 'SEDSNET_EMBEDDED_SCHEMA_PATH', 'DEVICE_IDENTIFIER'):
            self.assertNotIn(key, env)
        self.assertEqual(env['CARGO_BUILD_JOBS'], '2')
        self.assertNotEqual(env['CARGO_TARGET_DIR'], '/wrong')
        args = json.loads((self.control / 'argv.json').read_text())
        self.assertEqual(args[args.index('--target') + 1], 'unit-host')
        self.assertEqual(args[args.index('--crate-type') + 1], 'staticlib')

    def test_external_dependency_is_snapshotted_and_rebased(self):
        dependency = self.path / 'crc'
        (dependency / 'src').mkdir(parents=True)
        (dependency / 'Cargo.toml').write_text('[package]\nname="crc"\nversion="1.0.0"\n')
        (dependency / 'src/lib.rs').write_text('// original immutable input\n')
        with (self.source / 'Cargo.toml').open('a') as out:
            out.write('[dependencies]\ncrc={path="../crc"}\n')
        original = sdk.source_files(dependency)
        result = self.prepare()
        record = json.loads(Path(result['proof']).read_text())
        self.assertTrue(record['owned_path_rewrites'])
        manifest = (Path(result['source']) / 'Cargo.toml').read_text()
        self.assertIn('dependencies/', manifest)
        self.assertEqual(original, sdk.source_files(dependency))
        self.assertTrue(all(not Path(p).is_absolute() for p in record['cached_dependencies']))

    def test_cache_tamper_and_unproven_override_fail_closed(self):
        result = self.prepare()
        with self.assertRaisesRegex(RuntimeError, 'needs matching'):
            self.prepare(SEDSNET_TEST_NATIVE_ARCHIVE=result['archive'])
        header = Path(result['header'])
        header.write_text(header.read_text() + '// mutation\n')
        with self.assertRaisesRegex(RuntimeError, 'integrity'):
            self.prepare()

    def test_older_sdk_is_actionable_and_build_failure_is_preserved(self):
        manifest = self.source / 'Cargo.toml'
        manifest.write_text(manifest.read_text().replace('4.3.6', '4.3.5'))
        with self.assertRaisesRegex(RuntimeError, '>=4.3.6'):
            self.prepare()
        self.assertFalse((self.control / 'calls').exists())
        manifest.write_text(manifest.read_text().replace('4.3.5', '4.3.6'))
        with self.assertRaisesRegex(RuntimeError, 'retained log'):
            self.prepare(NATIVE_FIXTURE_FAIL='1')
        logs = list((self.root / 'build/host-sdk').glob('*.building-*/build.log'))
        self.assertEqual(len(logs), 1)
        self.assertFalse(list((self.root / 'build/host-sdk').glob('*/complete.json')))

    def test_singlequoted_and_escaped_dependency_paths_are_owned(self):
        dependency = self.path / 'crc quoted\\literal'
        dependency.mkdir()
        (dependency / 'Cargo.toml').write_text('[package]\nname="crc"\nversion="1.0.0"\n')
        for literal in ("'" + str(dependency) + "'", json.dumps(str(dependency))):
            with self.subTest(literal=literal):
                (self.source / 'Cargo.toml').write_text(
                    '[package]\nname="SEDSnet"\nversion="4.3.6"\n[dependencies]\ncrc={path=' + literal + '}\n')
                result = self.prepare()
                proof = json.loads(Path(result['proof']).read_text())
                self.assertTrue(proof['owned_path_rewrites'])
                for relative in proof['cached_dependencies']:
                    self.assertTrue((Path(result['proof']).parent / relative).is_relative_to(Path(result['proof']).parent))
        self.assertEqual((self.control / 'calls').read_text().splitlines(), ['build', 'build'])

    def test_unsupported_path_literal_fails_before_any_sdk_build(self):
        dependency = self.path / 'crc'
        dependency.mkdir()
        (dependency / 'Cargo.toml').write_text('[package]\nname="crc"\nversion="1.0.0"\n')
        with (self.source / 'Cargo.toml').open('a') as output:
            output.write('[dependencies]\ncrc={path=' + ("'" * 3) + '../crc' + ("'" * 3) + '}\n')
        with self.assertRaisesRegex(RuntimeError, 'Multiline'):
            self.prepare()
        self.assertFalse((self.control / 'calls').exists())

    def test_default_cache_uses_selector_with_configured_dev_and_offline(self):
        (self.root / 'cmake').mkdir()
        (self.root / 'cmake/sedsnet_source.py').write_text('# selector fixture\n')
        (self.root / 'cmake/sedsnet_source.cmake').write_text('set(SEDSNET_REPOSITORY "https://example.invalid/SDK")\n')
        (self.root / 'build/Release').mkdir(parents=True)
        old = self.path / 'old-sdk'
        old.mkdir()
        (old / 'Cargo.toml').write_text('[package]\nname="SEDSnet"\nversion="4.3.5"\n')
        (self.root / 'build/Release/CMakeCache.txt').write_text(
            '// Selected SDK source\n\nSEDSNET_DIR:PATH=' + str(old) + '\n\n// Configured branch\nSEDSNET_GIT_REF:STRING=dev\n\nSEDSNET_OFFLINE:BOOL=ON\n')
        with patch.object(sdk, 'run', return_value=str(self.source)) as selector:
            chosen = sdk.select_source(self.root, self.root / 'build/host-sdk', {})
        self.assertEqual(chosen, self.source)
        argv = selector.call_args.args[0]
        self.assertEqual(argv[argv.index('--ref') + 1], 'dev')
        self.assertIn('--offline', argv)
        self.assertIn(str(old), argv)
        with patch.object(sdk, 'run', side_effect=AssertionError('explicit source must never fetch')):
            self.assertEqual(sdk.select_source(self.root, self.root / 'build/host-sdk', self.env), self.source)

    def test_published_override_checks_ignored_cargo_lock(self):
        archive = self.path / 'public.a'
        archive.write_bytes(b'ORCHESTRATION UNIT MOCK')
        lock = self.source / 'Cargo.lock'
        lock.write_text('original ignored lock\n')
        package = self.path / 'package-proof.json'
        package.write_text(json.dumps({'release_clone_lock_sha256': sdk.sha(lock),
                                       'version': '4.3.6', 'VCS': {'git': {'sha1': 'unit-main'}}}))
        proof = self.path / 'public-proof.json'
        proof.write_text(json.dumps({'state': 'PUBLIC436_PACKAGE_VERIFICATION_NATIVE_ARCHIVE',
            'archive_sha256': sdk.sha(archive), 'header_sha256': sdk.sha(self.source / 'C-Headers/sedsnet.h'),
            'source': str(self.source), 'source_main': 'unit-main', 'package_proof': str(package)}))
        def git(argv, **kwargs):
            return 'unit-main' if 'rev-parse' in argv else ''
        with patch.object(sdk, 'run', side_effect=git):
            self.assertTrue(sdk.validate_override(self.source, archive, proof)['reused_verified_override'])
            lock.write_text('changed ignored lock\n')
            with self.assertRaisesRegex(RuntimeError, 'Cargo.lock'):
                sdk.validate_override(self.source, archive, proof)

    def test_concurrent_default_invocations_publish_only_one_complete_artifact(self):
        second_requested = threading.Event()
        count = 0
        original_lock = sdk.cache_lock
        @contextmanager
        def observed_lock(path):
            nonlocal count
            count += 1
            if count == 2:
                second_requested.set()
            with original_lock(path):
                yield
        with patch.object(sdk, 'cache_lock', observed_lock), ThreadPoolExecutor(2) as workers:
            first = workers.submit(self.prepare, NATIVE_FIXTURE_BARRIER='1')
            deadline = time.monotonic() + 5
            while not (self.control / 'started').exists():
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.001)
            second = workers.submit(self.prepare, NATIVE_FIXTURE_BARRIER='1')
            self.assertTrue(second_requested.wait(5))
            self.assertFalse(second.done())
            self.assertFalse(list((self.root / 'build/host-sdk').glob('*/complete.json')))
            (self.control / 'release').touch()
            self.assertEqual(first.result(5)['archive'], second.result(5)['archive'])
        self.assertEqual((self.control / 'calls').read_text().splitlines(), ['build'])

if __name__ == '__main__':
    unittest.main()
