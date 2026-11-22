import unittest

import numpy as np

from hinge_geometry import eye
from hinge_scoring import Engagement, HingeWitness, assess, flap_angle, score


class BoreTests(unittest.TestCase):
    def test_reset_poses_use_native_float_coordinates(self):
        from task import OBJECTS

        for obj in OBJECTS:
            self.assertTrue(all(type(value) is float for value in obj.pos))

    def test_open_bore_and_axial_chamfers(self):
        pieces = eye(.010, .0055, .013)
        points = np.concatenate(pieces)
        self.assertAlmostEqual(np.ptp(points[:, 1]), .010)
        self.assertAlmostEqual(points[:, 0].max(), .013)
        # Central faces leave the entire 11 mm hole open.
        for piece in pieces[16:32]:
            edge = piece[1, [0, 2]] - piece[0, [0, 2]]
            normal = np.array([edge[1], -edge[0]])
            normal /= np.linalg.norm(normal)
            self.assertAlmostEqual(abs(piece[0, [0, 2]] @ normal), .0055)

    def test_wide_flap_clears_the_outer_ears_when_seated(self):
        from task import FLAP
        from hinge_scoring import EAR_GAP_HALF

        for center, size, _ in FLAP.boxes:
            if abs(center[1]) + size[1] / 2 > EAR_GAP_HALF:
                self.assertGreater(center[0] - size[0] / 2, .013)


class ScoringTests(unittest.TestCase):
    def setUp(self):
        self.identity = np.eye(3)
        self.base = np.zeros(3)

    def measured(self, eye_center=(0., 0., 0.), pin_center=(0., -.0005, 0.)):
        return assess(self.base, self.identity, np.asarray(eye_center), self.identity,
                      np.asarray(pin_center), self.identity)

    def test_bypassed_flap_has_no_credit(self):
        self.assertEqual(score(self.measured(eye_center=(.030, 0., 0.)), True, True), 0.)
        self.assertEqual(score(self.measured(eye_center=(0., .035, 0.)), True, True), 0.)

    def test_pin_penetration_tiers_and_release(self):
        self.assertEqual(score(self.measured(pin_center=(0., -.052, 0.)), False, False), .25)
        self.assertEqual(score(self.measured(pin_center=(0., -.018, 0.)), False, False), .50)
        final = self.measured()
        self.assertTrue(final.captured)
        self.assertEqual(score(final, True, False), .75)
        self.assertEqual(score(final, True, True), 1.)

    def test_loss_of_far_ear_protrusion_revokes_full_engagement(self):
        self.assertFalse(self.measured(pin_center=(0., -.003, 0.)).captured)

    def test_requires_relative_rotation_and_continuous_capture(self):
        witness = HingeWitness()
        self.assertFalse(witness.update(self.measured(), 0.))
        self.assertFalse(witness.update(self.measured(), 0.))
        self.assertTrue(witness.update(self.measured(), np.deg2rad(16)))
        self.assertFalse(witness.update(Engagement(), np.deg2rad(20)))
        self.assertFalse(witness.update(self.measured(), np.deg2rad(20)))

    def test_whole_assembly_rotation_does_not_turn_the_flap(self):
        angle = np.deg2rad(20)
        c, s = np.cos(angle), np.sin(angle)
        tilted = np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])
        self.assertAlmostEqual(flap_angle(tilted, tilted), 0.)
        self.assertAlmostEqual(flap_angle(self.identity, tilted), angle)


if __name__ == "__main__":
    unittest.main()
