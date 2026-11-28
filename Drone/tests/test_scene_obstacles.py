from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers/wheels'))
from scene_obstacles import flight_boxes


class SceneObstacleTests(unittest.TestCase):
    def test_inserted_rotated_box_is_used_at_flight_height_and_removed_with_node(self):
        nodes = []
        children = SimpleNamespace(getCount=lambda: len(nodes), getMFNode=nodes.__getitem__)
        self.assertEqual(flight_boxes(children, (0, 0, 13)), [])
        geometry = SimpleNamespace(
            getBaseTypeName=lambda: 'Box',
            getField=lambda name: SimpleNamespace(getSFVec3f=lambda: [2, 4, 3]),
        )
        nodes.append(SimpleNamespace(
            getBaseTypeName=lambda: 'Solid',
            getPosition=lambda: (8, 0, 13),
            getOrientation=lambda: (0, -1, 0, 1, 0, 0, 0, 0, 1),
            getField=lambda name: SimpleNamespace(getSFNode=lambda: geometry),
        ))
        box, = flight_boxes(children, (0, 0, 13))
        self.assertEqual((box['x'], box['y'], box['sx'], box['sy']), (8, 0, 4, 2))
        self.assertEqual(flight_boxes(children, (0, 0, 3)), [])
        nodes.clear()
        self.assertEqual(flight_boxes(children, (0, 0, 13)), [])
