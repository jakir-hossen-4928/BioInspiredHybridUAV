import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from client_demo import relay


class ClientDemoTests(unittest.TestCase):
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
