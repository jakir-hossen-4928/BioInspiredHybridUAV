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
    def test_camera_windows_reopen_after_webots_saves_closed_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / 'farm.wbt'
            project = world.with_name('.farm.wbproj')
            project.write_bytes(b'Webots Project File version R2025a\r\n'
                                b'centralWidgetVisible: 1\r\n'
                                b'renderingDevicePerspectives: Drone:front;0;1;0;0\r\n')
            cameras = ('Drone:front', 'Drone:bottom')
            client_demo.configure_camera_windows(world, cameras)
            updated = project.read_bytes()
            self.assertIn(b'Drone:front;1;1;0;0;20;60;480;360;0\r\n', updated)
            self.assertIn(b'Drone:bottom;1;1;0;0;525;60;480;360;0\r\n', updated)
            self.assertIn(b'centralWidgetVisible: 1\r\n', updated)
            client_demo.configure_camera_windows(world, cameras)
            self.assertEqual(project.read_bytes(), updated)

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
