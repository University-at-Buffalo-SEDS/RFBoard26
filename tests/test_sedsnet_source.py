"""Exercise real Git updates and offline fallback without internet or hardware."""
import importlib.util
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sedsnet_source', ROOT / 'cmake/sedsnet_source.py')
source = importlib.util.module_from_spec(spec)
spec.loader.exec_module(source)


class SourceSelectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.remote = self.root / 'remote'
        source.git('init', '-b', 'main', self.remote)
        source.git('-C', self.remote, 'config', 'user.email', 'test@example.invalid')
        source.git('-C', self.remote, 'config', 'user.name', 'Test')
        for name in ('CMakeLists.txt', 'Cargo.toml', 'build.rs'):
            (self.remote / name).write_text('initial\n')
        self.commit()
        self.cache = self.root / 'cache'

    def commit(self):
        source.git('-C', self.remote, 'add', '.')
        source.git('-C', self.remote, '-c', 'commit.gpgsign=false', 'commit', '-m', 'fixture')

    def test_updates_main_without_destroying_prepared_old_source(self):
        first = source.select_source(self.cache, str(self.remote), [])
        (first / 'Cargo.toml').write_text('board preparation\n')
        (self.remote / 'build.rs').write_text('updated main\n')
        self.commit()
        second = source.select_source(self.cache, str(self.remote), [])
        self.assertNotEqual(first, second)
        self.assertEqual((second / 'build.rs').read_text(), 'updated main\n')
        self.assertEqual((first / 'Cargo.toml').read_text(), 'board preparation\n')
        self.assertEqual(source.select_source(self.cache, str(self.remote), []), second)

    def test_network_failure_uses_last_successful_source(self):
        first = source.select_source(self.cache, str(self.remote), [])
        self.assertEqual(source.select_source(self.cache, str(self.root / 'missing'), []), first)

    def test_offline_upgrade_uses_existing_checkout(self):
        self.assertEqual(source.select_source(self.cache, 'unavailable', [self.remote], offline=True), self.remote)
        self.assertEqual(source.select_source(self.cache, str(self.root / 'missing'), [self.remote]), self.remote)

    def test_cmake_reconfigure_retries_fetch_and_honors_override(self):
        project = self.root / 'project'
        (project / 'cmake').mkdir(parents=True)
        for name in ('sedsnet_source.cmake', 'sedsnet_source.py'):
            shutil.copyfile(ROOT / 'cmake' / name, project / 'cmake' / name)
        (project / 'cmake/prepare_sedsnet.cmake').write_text(
            'file(WRITE "${SEDSNET_SOURCE_DIR}/prepared" "yes")\n')
        (project / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.24)\nproject(source_test NONE)\ninclude(FetchContent)\ninclude(cmake/sedsnet_source.cmake)\nFetchContent_Declare(sedsnet GIT_REPOSITORY invalid GIT_TAG main)\nFetchContent_MakeAvailable(sedsnet)\nif(NOT EXISTS "${sedsnet_SOURCE_DIR}/prepared")\n message(FATAL_ERROR "Preparation missing")\nendif()\n')
        local = project / 'SEDSnet'
        local.mkdir()
        for name in ('CMakeLists.txt', 'Cargo.toml', 'build.rs'):
            (local / name).write_text('')
        def configure(*args):
            result = subprocess.run(['cmake', '-S', str(project), '-B', str(self.root / 'build'), *args],
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return result.stderr
        self.assertIn('offline/local', configure('-DSEDSNET_OFFLINE=ON'))
        self.assertIn('main unavailable', configure('-DSEDSNET_OFFLINE=OFF',
                      f'-DSEDSNET_REPOSITORY={self.root / "missing"}'))
        self.assertNotIn('main unavailable', configure(f'-DFETCHCONTENT_SOURCE_DIR_SEDSNET={local}'))

    def test_offline_without_source_fails_clearly(self):
        with self.assertRaisesRegex(RuntimeError, 'No usable SEDSnet source'):
            source.select_source(self.cache, 'unavailable', [], offline=True)

    def test_broken_online_main_does_not_silently_select_stale_version(self):
        first = source.select_source(self.cache, str(self.remote), [])
        (self.remote / 'Cargo.toml').unlink()
        self.commit()
        with self.assertRaisesRegex(RuntimeError, 'not a usable'):
            source.select_source(self.cache, str(self.remote), [])
        self.assertEqual(source.select_source(self.cache, '', [], offline=True), first)


if __name__ == '__main__':
    unittest.main()
