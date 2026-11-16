"""The RF service loop polls discovery and time sync exactly once per pass.

process_all_queues_timeout() must only drain queues: the SDK's
process_all_queues_with_timeout / periodic would poll both again inside the
same pass and double their cost.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def body(text, signature):
    start = text.index(signature)
    start = text.index("{", start)
    depth = 0
    for i in range(start, len(text)):
        depth += {"{": 1, "}": -1}.get(text[i], 0)
        if depth == 0:
            return text[start:i + 1]
    raise AssertionError(signature)


def strip_comments(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


class ServiceLoopPollTests(unittest.TestCase):
    def test_queue_service_drains_without_polling(self):
        text = strip_comments((ROOT / "Core/Src/telemetry.c").read_text())
        locked = body(text, "static SedsResult process_all_queues_timeout_locked")
        self.assertIn("seds_router_drain_queues_with_timeout(", locked)
        for polling in ("seds_router_process_all_queues_with_timeout", "seds_router_periodic",
                        "seds_router_poll_timesync", "seds_router_poll_discovery"):
            self.assertNotIn(polling, locked)

    def test_loop_polls_each_once_per_pass(self):
        text = strip_comments((ROOT / "Core/Src/telemetry_thread.c").read_text())
        entry = body(text, "void telemetry_thread_entry")
        loop = body(entry, "for (;;)")
        self.assertEqual(loop.count("telemetry_poll_discovery()"), 1)
        self.assertEqual(loop.count("telemetry_poll_timesync()"), 1)
        self.assertEqual(loop.count("process_all_queues_timeout("), 1)
        self.assertEqual(loop.count("tx_thread_sleep("), 1)


if __name__ == "__main__":
    unittest.main()
