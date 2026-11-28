import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import client_demo
from client_demo import relay


class ClientDemoTests(unittest.TestCase):
    @unittest.skipUnless(os.name == 'nt', 'Windows launcher')
    def test_stale_session_marker_does_not_block_next_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            active = folder / 'active.json'
            active.write_text('{"session":"stale"}', encoding='utf-8')
            with patch.multiple(client_demo, SESSIONS=folder, ACTIVE=active), \
                    patch.object(sys, 'argv', ['client_demo.py']), \
                    patch.object(client_demo, 'run_session', return_value=0) as run:
                self.assertEqual(client_demo.main(), 0)
                run.assert_called_once()
                self.assertFalse(active.exists())

    def test_relay_keeps_reference_pose_out_of_navigation(self):
        records = [
            ['truth', 0., [99., 99., 99.]],
            ['nav', 1., [1., 2., 3.]],
            ['map', 6., [[1., 2., 3.], [4., 5., 6.]]],
            ['truth', 7., [99., 99., 99.]],
        ]
        stream = SimpleNamespace(stdout=io.StringIO(''.join(json.dumps(r) + '\n' for r in records)))
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            relay(stream, folder)
            self.assertEqual(json.loads((folder / 'pose.json').read_text()), records[1])
            self.assertEqual(json.loads((folder / 'ros-live.json').read_text())['map_points'], 2)
            self.assertEqual(len((folder / 'ros.jsonl').read_text().splitlines()), len(records))
