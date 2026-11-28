"""Stereo depth tests on synthetic rectified images; no Webots required."""
import math
import sys
import unittest
from pathlib import Path

C = Path(__file__).resolve().parents[1] / 'controllers' / 'drone'
sys.path.insert(0, str(C))
from oak_stereo import objects

W, H, FOV = 320, 200, 1.3962634
BASELINE = .075
FOCAL = W / (2 * math.tan(FOV / 2))
SIZE = 40


def texture(offset, y):
    """Object-attached pattern: matching must compare the same surface point."""
    return 30 + ((offset * 7 + y * 13) % 160)


def gray_image(edge, textured=True):
    rows = []
    for y in range(H):
        row = bytearray()
        for x in range(W):
            if edge <= x < edge + SIZE:
                g = texture(x - edge, y) if textured else 70
            else:
                g = 90
            row += bytes((g, g, g, 255))
        rows.append(bytes(row))
    return b''.join(rows)


def red_image(edge):
    rows = []
    for y in range(H):
        row = bytearray()
        for x in range(W):
            row += bytes((0, 0, 220, 255)) if edge <= x < edge + SIZE else bytes((90, 90, 90, 255))
        rows.append(bytes(row))
    return b''.join(rows)


class FakeCamera:
    def __init__(self, buffer):
        self.buffer = buffer
    def getWidth(self):
        return W
    def getHeight(self):
        return H
    def getFov(self):
        return FOV
    def getImage(self):
        return self.buffer


def views(disparity, edge=100):
    return (FakeCamera(red_image(edge)),
            FakeCamera(gray_image(edge)),
            FakeCamera(gray_image(edge - disparity)))


class StereoDepthTests(unittest.TestCase):
    def test_disparity_becomes_metric_depth(self):
        disparity = 10
        hits = objects(*views(disparity), mount_x=.72)
        self.assertEqual(len(hits), 1, hits)
        distance, lateral, width, measured = hits[0]
        self.assertAlmostEqual(measured, disparity, delta=.6)
        self.assertAlmostEqual(distance, FOCAL * BASELINE / disparity + .72, delta=.25)
        self.assertGreater(width, .1)

    def test_flat_coloured_obstacle_still_gets_exact_disparity(self):
        # The real arena obstacle is one flat colour, which ties every disparity
        # that keeps the silhouette inside the shifted copy. Sampling the last
        # column is what breaks the tie; without it the depth is ~1 m short.
        for disparity in (4, 7, 12):
            rgb, left, _ = views(disparity)
            right = FakeCamera(gray_image(100 - disparity, textured=False))
            left = FakeCamera(gray_image(100, textured=False))
            hits = objects(rgb, left, right, mount_x=0.)
            self.assertEqual(len(hits), 1, (disparity, hits))
            self.assertAlmostEqual(hits[0][3], disparity, delta=.1)
            self.assertAlmostEqual(hits[0][0], FOCAL * BASELINE / disparity, delta=.05)

    def test_no_depth_without_a_detection(self):
        plain = FakeCamera(gray_image(W))
        self.assertEqual(objects(plain, plain, plain, 0.), [])

    def test_depth_moves_towards_camera_as_disparity_grows(self):
        near = objects(*views(10), mount_x=0.)[0][0]
        far = objects(*views(5), mount_x=0.)[0][0]
        self.assertLess(near, far)


if __name__ == '__main__':
    unittest.main()
