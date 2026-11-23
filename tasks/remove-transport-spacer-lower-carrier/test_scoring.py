import json
import unittest

import numpy as np

from scoring import Trial


class CarrierScoring(unittest.TestCase):
    def test_legitimate_removal_descent_and_release(self):
        t = Trial()
        self.assertEqual(t.score(.036, .013, False, False, False, False), .25)
        self.assertEqual(t.score(.036, .096, True, True, False, False), .5)
        self.assertEqual(t.score(.002, .096, True, True, True, False), .75)
        self.assertEqual(t.score(.001, .096, True, True, True, True), 1.)

    def test_late_speed_violation_revokes_success(self):
        t = Trial()
        t.sample(.02, .02, .09, .005)
        self.assertEqual(t.score(.036, .096, True, True, False, False), .5)
        t.sample(.106, .001, .09, .005)
        self.assertEqual(t.score(.001, .096, True, True, True, True), 0.)

    def test_native_samples_keep_the_diagnostic_flags_serializable(self):
        t = Trial()
        t.sample(np.float64(0.), np.float64(.036), .09, np.float64(.005))
        t.sample(np.float64(0.), np.float64(.0359), .09, np.float64(.005))
        self.assertEqual(json.dumps(t.dropped), "false")

    def test_abrupt_teleport(self):
        t = Trial()
        t.sample(0., .036, .09, .005)
        t.sample(0., 0., .09, .005)
        self.assertEqual(t.score(0., .096, True, True, True, True), 0.)

    def test_lowering_while_spacer_is_still_underneath(self):
        t = Trial()
        t.sample(.02, .029, .023, .005)
        self.assertEqual(t.score(.001, .04, False, False, True, True), 0.)
