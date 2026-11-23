import math
import unittest

import numpy as np

from scoring import Pose, evaluate

W = .34
TOP = .7475


def pose(x, y, z, yaw=0.):
    c, s = math.cos(yaw), math.sin(yaw)
    return Pose(np.array([x, y, z]), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))


def trial(y=.005, x=W, yaw=0., block_z=TOP+.008, released=False, block_x=None):
    tray = pose(x, y, TOP+.004, yaw)
    sleeve = pose(W, 0., TOP)
    block = pose(x if block_x is None else block_x, y, block_z)
    return evaluate(tray, sleeve, block, TOP, W, released)[0]


class SleeveScoring(unittest.TestCase):
    def test_insertion_thresholds_and_release(self):
        self.assertEqual([trial(y) for y in (.208, .192, .160, .102, .006)], [0, 0, .25, .5, .75])
        self.assertEqual(trial(released=True), 1.)

    def test_product_left_at_reset_or_lost_late(self):
        self.assertEqual(trial(block_x=.21), 0.)
        self.assertEqual(trial(block_z=TOP-.05, released=True), 0.)

    def test_product_projecting_above_rim_is_not_contained(self):
        self.assertEqual(trial(y=.102, block_z=TOP+.029), 0.)

    def test_skewed_and_incorrect_side(self):
        self.assertEqual(trial(y=.10, yaw=math.radians(17)), 0.)
        self.assertEqual(trial(y=-.18), 0.)
        self.assertEqual(trial(x=W+.067), 0.)
