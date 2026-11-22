"""Static controls for tray seating, release and final-state scoring."""
import unittest

import numpy as np

from scoring import Dwell, Inputs, score


class ScoreTests(unittest.TestCase):
    def test_each_tier_and_current_state(self):
        dwell = Dwell()
        self.assertEqual(score(Inputs(), dwell, 1), 0)
        self.assertEqual(score(Inputs(unlocked=True), dwell, 2), .25)
        self.assertEqual(score(Inputs(separate=True), dwell, 3), .5)
        self.assertEqual(score(Inputs(separate=True, upright=True), dwell, 4), .75)
        self.assertEqual(score(Inputs(), dwell, 5), 0)

    def test_full_score_requires_distinct_stable_released_steps(self):
        dwell = Dwell()
        poses = (np.zeros(3), np.eye(3), np.ones(3), np.eye(3))
        staged = Inputs(separate=True, upright=True, staged=True, clear=True, poses=poses)
        for _ in range(30):
            self.assertEqual(score(staged, dwell, 1), .75)
        for step in range(2, 20):
            self.assertEqual(score(staged, dwell, step), .75)
        self.assertEqual(score(staged, dwell, 20), 1)
        self.assertEqual(score(Inputs(separate=True, upright=True), dwell, 21), .75)

    def test_off_table_failure_latches(self):
        dwell = Dwell()
        self.assertEqual(score(Inputs(invalid=True, separate=True), dwell, 1), 0)
        self.assertEqual(score(Inputs(separate=True, upright=True), dwell, 2), 0)

    def test_motion_resets_dwell(self):
        dwell = Dwell()
        for step in range(1, 25):
            poses = (np.array([step * .003, 0., 0.]), np.eye(3), np.ones(3), np.eye(3))
            staged = Inputs(separate=True, upright=True, staged=True, clear=True, poses=poses)
            self.assertEqual(score(staged, dwell, step), .75)


class MechanicsTests(unittest.TestCase):
    def test_nominal_slide_clears_both_lips_before_lift(self):
        from task import INNER, OUTER, RELEASE, ROOFS, TONGUES

        reset_height = INNER.pos[2] - OUTER.pos[2]

        def intersects(tab, roof, dx, dz):
            moved = tab + [dx, 0., dz]
            overlap = np.minimum(moved.max(axis=0), roof.max(axis=0)) - np.maximum(
                moved.min(axis=0), roof.min(axis=0)
            )
            return bool(np.all(overlap > .00001))

        for tab, roof in zip(TONGUES, ROOFS):
            self.assertTrue(intersects(tab, roof, 0., reset_height + .010))
            for rise in np.linspace(0, .060, 61):
                self.assertFalse(intersects(tab, roof, RELEASE, reset_height + rise))

    def test_tip_penetration_is_not_a_release(self):
        from scoring import overlap_depth

        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        ledge = .002 * signs
        tab = .002 * signs
        self.assertGreater(overlap_depth(tab + [.001, 0., 0.], ledge, np.eye(3)), .0025)
        self.assertLess(overlap_depth(tab + [.0035, 0., 0.], ledge, np.eye(3)), .001)
        self.assertLess(overlap_depth(tab + [.005, 0., 0.], ledge, np.eye(3)), 0.)

    def test_geometry_distinguishes_nested_floating_and_inverted(self):
        from partkit import PartState
        from task import INNER, OUTER, PAD_H, PAD_X, PAD_Y, TOP, measure

        def state(pos, quaternion=(0., 0., 0., 1.)):
            return PartState(pos=np.array(pos), quat_xyzw=np.array(quaternion), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))

        distant_hand = [(np.ones(3), np.full(3, 1.1))]
        nested = {OUTER.name: state(OUTER.pos), INNER.name: state(INNER.pos)}
        measured, _ = measure(nested, distant_hand)
        self.assertFalse(measured.separate)
        self.assertFalse(measured.unlocked)

        outer = state((PAD_X, -PAD_Y, TOP + PAD_H))
        floating = state((PAD_X, PAD_Y, TOP + PAD_H + .005))
        measured, _ = measure({OUTER.name: outer, INNER.name: floating}, distant_hand)
        self.assertTrue(measured.separate)
        self.assertTrue(measured.upright)
        self.assertFalse(measured.staged)

        inverted = state((PAD_X, PAD_Y, TOP + PAD_H + .068), (1., 0., 0., 0.))
        measured, _ = measure({OUTER.name: outer, INNER.name: inverted}, distant_hand)
        self.assertTrue(measured.separate)
        self.assertFalse(measured.upright)
