import unittest

import numpy as np

from chip_scoring import count_captured, score


class ChipsTest(unittest.TestCase):
    def test_count_requires_footprint_and_floor_support(self):
        valid = np.array([0., 0., .006])
        points = [valid, valid+[.063,0,0], valid+[0,.052,0], valid-[0,0,.005], valid+[0,0,.05]]
        self.assertEqual(count_captured(points), 1)

    def test_credit_revokes_after_chips_escape(self):
        self.assertEqual(score(10, True, True), 1.)
        self.assertEqual(score(9, True, True), .75)
        self.assertEqual(score(5, True, True), .50)
        self.assertEqual(score(3, True, True), .25)
        self.assertEqual(score(1, True, True), 0.)

    def test_requires_release_tool_parking_and_valid_reset_count(self):
        self.assertEqual(score(10, True, False), .75)
        self.assertEqual(score(10, False, True), 0.)
