import unittest
from unittest import mock
import build


class DefaultStorageTests(unittest.TestCase):
    def test_build_flash_and_full_test_select_qualified_storage_by_default(self):
        for command in ("build", "flash", "test"):
            with self.subTest(command=command):
                args = build.make_parser().parse_args([command, "--release"])
                self.assertEqual((args.allocator, args.packet_store), ("tlsf", "compact"))
                if hasattr(build, "build_cfg_from_args"):
                    cfg = build.build_cfg_from_args(mock.Mock(), args)
                    self.assertEqual((cfg.allocator, cfg.packet_store), ("tlsf", "compact"))
                else:
                    _, options = build.cli_build_options(args)
                    self.assertTrue(options["allocator-tlsf"])
                    self.assertTrue(options["packet-store-compact"])

    def test_explicit_threadx_and_heap_remain_available(self):
        args = build.make_parser().parse_args(["build", "--allocator", "threadx", "--packet-store", "heap"])
        if hasattr(build, "build_cfg_from_args"):
            cfg = build.build_cfg_from_args(mock.Mock(), args)
            self.assertEqual((cfg.allocator, cfg.packet_store), ("threadx", "heap"))
        else:
            _, options = build.cli_build_options(args)
            self.assertFalse(options["allocator-tlsf"])
            self.assertFalse(options["packet-store-compact"])
