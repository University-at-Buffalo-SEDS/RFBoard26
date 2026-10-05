import argparse
import unittest
import build

class CommonCliTests(unittest.TestCase):
    def test_all_options_have_help(self):
        def visit(parser):
            for action in parser._actions:
                if action.option_strings:
                    self.assertTrue(action.help, action.option_strings)
                if isinstance(action, argparse._SubParsersAction):
                    for child in action.choices.values(): visit(child)
        visit(build.make_parser())

    def test_common_build_flash_and_test_flags(self):
        for command in ('build', 'flash', 'test'):
            args = build.make_parser().parse_args([command, '--release', '--allocator', 'tlsf', '--packet-store', 'compact'])
            self.assertEqual(args.allocator, 'tlsf')
            self.assertEqual(args.packet_store, 'compact')
            self.assertTrue(args.watchdog)
            args = build.make_parser().parse_args([command, '--no-watchdog', '--sedsnet-ref', 'main'])
            self.assertFalse(args.watchdog)
            self.assertEqual(args.sedsnet_ref, 'main')

    def test_fc_flag_translation(self):
        if not hasattr(build, 'cli_build_options'): return
        args = build.make_parser().parse_args(['build','--release','--allocator','tlsf','--packet-store','compact','--sedsnet-ref','main','--no-watchdog','--no-gps'])
        preset,options=build.cli_build_options(args)
        self.assertEqual(preset,'Release')
        self.assertTrue(options['allocator-tlsf'])
        self.assertTrue(options['packet-store-compact'])
        self.assertEqual(options['sedsnet-ref'],'main')
        self.assertTrue(options['no-watchdog'])
        self.assertTrue(options['nogps'])
        args=build.make_parser().parse_args(['--build-subdir','custom','build','--toolchain','custom.cmake'])
        self.assertEqual(args.build_subdir,'custom')
        self.assertEqual(args.toolchain,'custom.cmake')
