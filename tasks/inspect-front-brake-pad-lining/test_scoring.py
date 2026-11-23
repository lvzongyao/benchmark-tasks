import unittest

from scoring import Measurement


class LiningMeasurementTests(unittest.TestCase):
    def update(self, state, tick, *, floor=False, roof=False, stop=False,
               intact=True, parked=False):
        return state.update(tick, floor, roof, stop, intact, parked)

    def test_fit_and_physical_stop_survive_park(self):
        state = Measurement()
        for tick in range(1, 5):
            self.assertEqual(self.update(state, tick, floor=True), .25)
        for tick in range(5, 9):
            result = self.update(state, tick, roof=True)
        self.assertEqual(result, .50)
        for tick in range(9, 13):
            result = self.update(state, tick, stop=True)
        self.assertEqual(result, .75)
        self.assertEqual(self.update(state, 13, parked=True), 1.)

    def test_floor_contact_and_proximity_cannot_pass(self):
        state = Measurement()
        for tick in range(1, 20):
            self.update(state, tick, floor=True, parked=True)
        self.assertEqual(state.score, .25)

    def test_fit_cannot_credit_wide_fixture_without_stop(self):
        state = Measurement()
        for tick in range(1, 12):
            self.update(state, tick, floor=True, roof=True, parked=True)
        self.assertEqual(state.score, .50)

    def test_duplicate_samples_cannot_complete_contact_dwell(self):
        state = Measurement()
        self.update(state, 1, floor=True, roof=True, stop=True)
        for _ in range(8):
            self.update(state, 1, floor=True, roof=True, stop=True, parked=True)
        self.assertLess(state.score, .75)

    def test_lost_tool_revokes_reading(self):
        state = Measurement()
        for tick in range(1, 5):
            self.update(state, tick, floor=True, roof=True, stop=True)
        self.assertEqual(self.update(state, 5, intact=False, parked=True), 0.)


class MeasuringSurfacesTests(unittest.TestCase):
    def test_tip_contacts_are_separate_from_stem_contacts(self):
        from types import SimpleNamespace
        import numpy as np
        import task
        import geometry as geo

        def pose(point):
            return SimpleNamespace(pos=np.asarray(point), rotation=lambda: np.eye(3))

        rotor, pad = pose(task.ROTOR), pose(task.PAD)
        tool = pose(task.PAD+[geo.EAR_X,-.068,.004])
        floor = tool.pos+[0.,0.,-.004]
        roof = tool.pos+[0.,0.,task.LINING-.004]
        stem = floor+[0.,-.010,0.]
        axis = np.array([0.,0.,1.])
        native = SimpleNamespace(pairs=[[
            [(0.2, floor, axis), (9.0, stem, axis)],
            [(0.3, roof, -axis)],
        ]])
        self.assertEqual(task.forces(0, tool, pad, rotor, native), (.2,.3,0.))
