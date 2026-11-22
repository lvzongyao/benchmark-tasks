import unittest

import numpy as np

from packing import Pose, assess, score


def pose(x, y, z):
    return Pose(np.array((x, y, z)), np.eye(3))


class PackingTests(unittest.TestCase):
    def setUp(self):
        self.tray = pose(0., 0., 0.)
        self.product = pose(0., 0., .008)
        self.left = pose(0., .0665, .004)
        self.right = pose(0., -.0665, .004)

    def test_bare_product_in_tray_has_no_credit(self):
        caps = (pose(0., .15, 0.), pose(0., -.15, 0.))
        self.assertEqual(score(assess(self.product, caps, self.tray), False), 0.)

    def test_wrong_end_and_caps_above_or_beside_product_do_not_count(self):
        for cap in (self.right, pose(.048, .0665, .004), pose(0., .0665, .024)):
            caps = (cap, self.left)
            self.assertEqual(score(assess(self.product, caps, self.tray), False), 0.)

    def test_one_and_two_protectors_before_transfer(self):
        origin = pose(-.16, 0., .08)
        caps = (pose(-.16, .0665, .076), pose(-.16, -.0665, .076))
        self.assertEqual(score(assess(origin, (caps[0], self.right), self.tray), True), .25)
        self.assertEqual(score(assess(origin, caps, self.tray), True), .50)

    def test_held_arrangement_cannot_score_full(self):
        state = assess(self.product, (self.left, self.right), self.tray)
        self.assertIs(type(state.protected), int)
        self.assertEqual(score(state, False), .75)
        self.assertEqual(score(state, True), 1.)

    def test_backed_out_protector_revokes_credit(self):
        backed_out = pose(0., -.075, .004)
        self.assertEqual(score(assess(self.product, (self.left, backed_out), self.tray), True), .25)

    def test_lateral_displacement_is_measured_in_tray_frame(self):
        tray = pose(.010, 0., 0.)
        self.assertEqual(score(assess(self.product, (self.left, self.right), tray), True), .50)

    def test_product_dropped_between_caps_has_no_credit(self):
        product_on_tray_floor = pose(0., 0., .004)
        self.assertEqual(score(assess(product_on_tray_floor, (self.left, self.right), self.tray), True), 0.)
