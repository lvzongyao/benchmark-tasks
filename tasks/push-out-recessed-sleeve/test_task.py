import unittest
from pathlib import Path

import numpy as np

from sleeve_geometry import annulus
from sleeve_scoring import State, score


class GeometryTests(unittest.TestCase):
    def test_scene_loads_with_three_independent_free_parts(self):
        from yam_arena.task_file import load

        task = load(Path(__file__).with_name("task.py"))
        self.assertEqual({obj.name for obj in task.objects if obj.free}, {"plate", "sleeve", "rod"})
        self.assertEqual(set(task.demo()), {"left", "right"})

    def test_left_grasp_pose_stays_fixed_during_rod_motion(self):
        from task import demo
        from yam_arena.demo import FINGER_CLOSED

        left = demo()["left"]
        held = [point for point in left if point.finger == FINGER_CLOSED]
        self.assertEqual({(point.pos, point.yaw) for point in held}, {(held[0].pos, held[0].yaw)})

    def test_left_grasp_descends_into_the_handle_face(self):
        from task import BORE, PLATE, demo
        from yam_arena.demo import FINGER_CLOSED

        grip = next(point for point in demo()["left"] if point.finger == FINGER_CLOSED)
        center, size, _ = PLATE.boxes[1]
        grip_on_face = np.array(grip.pos) - BORE - center
        self.assertGreater(size[2] / 2 - abs(grip_on_face[2]), .015)
        self.assertGreater(size[1] / 2 - abs(grip_on_face[1]), .008)

    def test_bore_has_complete_convex_wall_and_open_axis(self):
        pieces = annulus(.020, .012, outer_half=(.060, .045))
        normal_minima = []
        for piece in pieces:
            face = piece[:4, 1:]
            tangent = face[1] - face[0]
            normal = np.array([tangent[1], -tangent[0]])
            normal /= np.linalg.norm(normal)
            normal_minima.append(np.min(face @ normal))
            self.assertLess(np.linalg.det(np.stack([face[1] - face[0], face[2] - face[1]])), 0)
        np.testing.assert_allclose(normal_minima, .012, atol=1e-10)
        cloud = np.concatenate(pieces)
        np.testing.assert_allclose(np.ptp(cloud, axis=0), [.020, .120, .090])

    def test_sleeve_outer_wall_leaves_radial_clearance(self):
        points = np.concatenate(annulus(.016, .005, outer_radius=.0119))
        self.assertAlmostEqual(np.max(np.linalg.norm(points[:, 1:], axis=1)), .0119)
        np.testing.assert_allclose(np.unique(points[:, 0]), [-.008, .008])


class ScoreTests(unittest.TestCase):
    def test_dwell_limits_relative_motion_over_the_entire_window(self):
        from task import stable

        reset = (np.zeros((3, 3)), np.repeat(np.eye(3)[None], 3, axis=0))
        self.assertTrue(stable(reset, reset))
        drift = (np.full((3, 3), .0019), reset[1])
        self.assertFalse(stable(reset, drift))
        angle = np.deg2rad(2.1)
        rotation = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                             [0., np.sin(angle), np.cos(angle)]])
        self.assertFalse(stable(reset, (reset[0], np.repeat(rotation[None], 3, axis=0))))

    def test_progress_requires_current_displacement(self):
        self.assertEqual(score(State()), 0)
        self.assertEqual(score(State(travel=.004, rear_projection=.002)), .25)
        self.assertEqual(score(State(travel=.010, rear_projection=.008)), .50)

    def test_capture_and_full_release(self):
        centered = dict(travel=.06, rear_projection=.05, outside=True, in_tray=True)
        self.assertEqual(score(State(**centered)), .75)
        self.assertEqual(score(State(**centered, empty_bore=True, rod_parked=True, plate_stable=True)), .75)
        self.assertEqual(score(State(**centered, empty_bore=True, rod_parked=True, plate_stable=True, released=True)), 1)
        self.assertEqual(score(State(**centered, lost=True)), 0)
        self.assertEqual(score(State(travel=.06, rear_projection=.05, outside=True)), .50)


if __name__ == "__main__":
    unittest.main()
