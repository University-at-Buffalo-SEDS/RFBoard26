import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import build
from build_cleanup import clean_generated


class BuildCleanupTests(unittest.TestCase):
    def clean(self, root, selected=None):
        if hasattr(build, 'clean_build'):
            ui = mock.Mock()
            build.clean_build(ui, root, selected)
            return '\n'.join(call.args[1] for call in ui.say.call_args_list)
        output = io.StringIO()
        with mock.patch.object(build, 'BUILDDIR', root / 'build'), contextlib.redirect_stdout(output):
            build.clean(root / 'build' / selected if selected else root / 'build')
        return output.getvalue()

    def put(self, path, content=b'generated'):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        return path

    def test_selected_clean_reports_space_and_preserves_other_build_and_crc_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            debug = self.put(root / 'build/Debug/firmware.elf')
            release = self.put(root / 'build/Release/firmware.elf')
            crc = root / 'third_party/embedded-crc32fast'
            self.put(crc / 'target/debug/libcrc32fast.rlib')
            self.put(crc / 'Cargo.lock')
            manifest = self.put(crc / 'Cargo.toml', b'keep manifest')
            source = self.put(crc / 'src/lib.rs', b'keep implementation')
            output = self.clean(root, 'Debug')
            self.assertFalse(debug.exists())
            self.assertTrue(release.exists())
            self.assertFalse((crc / 'target').exists())
            self.assertFalse((crc / 'Cargo.lock').exists())
            self.assertEqual(manifest.read_bytes(), b'keep manifest')
            self.assertEqual(source.read_bytes(), b'keep implementation')
            self.assertIn('Reclaimed approximately', output)
            self.assertRegex(output, r'\([1-9][0-9,]* allocated file bytes\)')

    def test_crc_only_artifacts_are_cleaned_even_when_build_is_absent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = self.put(root / 'third_party/embedded-crc32fast/target/release/output')
            self.clean(root)
            self.assertFalse(artifact.exists())
            output = self.clean(root)
            self.assertIn('Already clean', output)
            self.assertIn('0 B (0 allocated file bytes)', output)

    def test_outside_target_is_rejected_before_crc_removal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outside = self.put(root / 'outside/keep')
            crc = self.put(root / 'third_party/embedded-crc32fast/target/keep')
            with self.assertRaisesRegex(ValueError, 'outside'):
                clean_generated(root / 'build', root / 'build/../outside')
            self.assertTrue(outside.exists())
            self.assertTrue(crc.exists())

    def test_symlinked_crc_parent_is_rejected_before_build_removal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = self.put(root / 'build/output')
            outside = root / 'outside'
            outside.mkdir()
            (root / 'third_party').symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'symbolic link'):
                clean_generated(root / 'build', root / 'build')
            self.assertTrue(artifact.exists())
            self.assertTrue(outside.exists())

    def test_nested_symlink_does_not_count_or_remove_external_data(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            external = self.put(root / 'external/keep', b'keep data')
            (root / 'build').mkdir()
            (root / 'build/link').symlink_to(external)
            _, reclaimed = clean_generated(root / 'build', root / 'build')
            self.assertEqual(reclaimed, 0)
            self.assertEqual(external.read_bytes(), b'keep data')

    def test_hard_links_are_counted_once_and_retained_links_free_no_blocks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = self.put(root / 'build/one')
            os.link(artifact, root / 'build/two')
            info = artifact.stat()
            expected = info.st_blocks * 512 if hasattr(info, 'st_blocks') else info.st_size
            _, reclaimed = clean_generated(root / 'build', root / 'build')
            self.assertEqual(reclaimed, expected)
            retained = self.put(root / 'retained')
            (root / 'build').mkdir()
            os.link(retained, root / 'build/shared')
            _, reclaimed = clean_generated(root / 'build', root / 'build')
            self.assertEqual(reclaimed, 0)
            self.assertTrue(retained.exists())


if __name__ == '__main__':
    unittest.main()
