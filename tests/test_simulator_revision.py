"""The simulation runner refuses a simulator image older than this board needs.

local-dev-v<version> is reused across FirmwareSimulator revisions. A stale
image on one host reported a DAQ factory boot hang that did not exist, so the
runner checks the image's revision label (or, unlabelled, its creation time)
against SIMULATOR_REQUIRED_COMMIT_TIME before running anything.
"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from sim import run_full

LOCAL = f"seds-firmware-simulator:local-dev-v{run_full.SIMULATOR_INTERFACE_VERSION}"
OLD = "2026-11-09T17:50:39.435011788-05:00"   # before the required revision
NEW = "2026-11-11T07:22:01Z"


def docker(images):
    """subprocess.run stand-in: no registry, and `images` maps tag -> (labels, created)."""
    def run(command, **kwargs):
        if command[1] == "pull":
            return mock.Mock(returncode=1, stdout="", stderr="denied")
        if command[1:3] == ["image", "inspect"]:
            image = command[-1]
            if image not in images:
                return mock.Mock(returncode=1, stdout="", stderr="missing")
            labels, created = images[image]
            if "--format" not in command:
                return mock.Mock(returncode=0, stdout="", stderr="")
            template = command[command.index("--format") + 1]
            text = json.dumps(labels) if "Labels" in template else created
            return mock.Mock(returncode=0, stdout=text + "\n", stderr="")
        raise AssertionError(command)
    return run


def labelled(time, revision="8dcf6786205930e0ff34966db39f909f4fd7536e"):
    return {"org.opencontainers.image.revision": revision,
            run_full.SIMULATOR_COMMIT_TIME_LABEL: time}


class SimulatorRevisionTests(unittest.TestCase):
    def resolve(self, images, environment=None):
        with mock.patch.dict(run_full.os.environ, environment or {}, clear=True):
            with mock.patch.object(run_full.subprocess, "run", side_effect=docker(images)):
                return run_full.resolve_simulator_image(mock.Mock(), "/usr/bin/docker", Path("/board"), "x")

    def test_required_revision_is_at_least_the_framed_probe_simulator(self):
        self.assertGreaterEqual(run_full.SIMULATOR_REQUIRED_COMMIT_TIME.isoformat(),
                                "2026-11-10T04:49:55+00:00")

    def test_local_fallback_by_revision_label(self):
        self.assertEqual(self.resolve({LOCAL: (labelled(NEW), OLD)}), LOCAL)
        with self.assertRaisesRegex(RuntimeError, "older than FirmwareSimulator"):
            self.resolve({LOCAL: (labelled("2026-11-09T22:00:00Z"), NEW)})

    def test_unlabelled_image_is_judged_by_creation_time(self):
        unlabelled = {"org.opencontainers.image.source": "https://github.com/x"}
        self.assertEqual(self.resolve({LOCAL: (unlabelled, NEW)}), LOCAL)
        with self.assertRaisesRegex(RuntimeError, "no revision label, built 2026-11-09T17:50:39"):
            self.resolve({LOCAL: (unlabelled, OLD)})
        with self.assertRaisesRegex(RuntimeError, "older than"):
            self.resolve({LOCAL: (None, OLD)})

    def test_explicit_image_is_checked_and_can_be_overridden(self):
        explicit = {"SEDS_FIRMWARE_SIM_IMAGE": "old:image"}
        images = {"old:image": ({}, OLD)}
        with self.assertRaisesRegex(RuntimeError, "SEDS_FIRMWARE_SIM_ALLOW_OLD=1"):
            self.resolve(images, explicit)
        with mock.patch("sys.stderr"):
            self.assertEqual(self.resolve(images, {**explicit, "SEDS_FIRMWARE_SIM_ALLOW_OLD": "1"}), "old:image")
        self.assertEqual(self.resolve({"new:image": (labelled(NEW), NEW)},
                                      {"SEDS_FIRMWARE_SIM_IMAGE": "new:image"}), "new:image")

    def test_source_build_labels_the_checkout_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            git = ["git", "-C", str(source)]
            environment = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.com",
                           "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.com",
                           "GIT_AUTHOR_DATE": "2026-11-11T02:22:01-05:00",
                           "GIT_COMMITTER_DATE": "2026-11-11T02:22:01-05:00", "PATH": "/usr/bin:/bin"}
            subprocess.run([*git, "init", "-q"], check=True)
            (source / "Dockerfile").write_text("FROM scratch\n")
            subprocess.run([*git, "add", "Dockerfile"], check=True)
            subprocess.run([*git, "commit", "-q", "-m", "x"], check=True, env=environment)
            head = subprocess.check_output([*git, "rev-parse", "HEAD"], text=True).strip()
            calls = []
            fake = docker({LOCAL: (labelled(NEW, head), NEW)})

            def run(command, *args, **kwargs):
                if command[1] == "build":
                    calls.append(command)
                    return mock.Mock(returncode=0)
                return fake(command, **kwargs)
            with mock.patch.dict(run_full.os.environ, {"SEDS_FIRMWARE_SIM_SOURCE": str(source)}, clear=True):
                with mock.patch.object(run_full.subprocess, "run", side_effect=run):
                    self.assertEqual(run_full.resolve_simulator_image(mock.Mock(), "/usr/bin/docker",
                                                                      Path("/board"), "x"), LOCAL)
        self.assertIn(f"SOURCE_REVISION={head}", calls[0])
        self.assertIn("SOURCE_COMMIT_TIME=2026-11-11T07:22:01Z", calls[0])


if __name__ == "__main__":
    unittest.main()
