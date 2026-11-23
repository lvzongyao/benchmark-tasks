import unittest

from secondary_lock_checks import Unlock, evaluate


class InterlockCases(unittest.TestCase):
    def test_both_locks_must_clear_before_withdrawal(self):
        for safety, latch in ((0., 0.), (.006, .006), (.012, .002)):
            lock = Unlock()
            lock.update(safety, latch, .005, True)
            lock.update(safety, latch, .013, True)
            self.assertTrue(lock.invalid)
            self.assertFalse(lock.escaped)

    def test_returning_to_reset_requires_a_new_unlock(self):
        lock = Unlock()
        lock.update(.011, .005, .005, True)
        lock.update(.011, .005, .013, True)
        self.assertTrue(lock.escaped)
        lock.update(0., 0., 0., True)
        self.assertFalse(lock.escaped)
        lock.update(0., 0., .013, True)
        self.assertTrue(lock.invalid)

    def test_progress_and_release(self):
        self.assertEqual(evaluate(False, False, 0., False, False, False, 0, False), 0.)
        self.assertEqual(evaluate(True, False, 0., False, False, False, 0, False), .25)
        self.assertEqual(evaluate(True, True, .005, False, False, False, 0, False), .50)
        self.assertEqual(evaluate(False, False, .060, True, True, True, 19, False), .75)
        self.assertEqual(evaluate(False, False, .060, True, True, True, 20, False), 1.)
        self.assertEqual(evaluate(False, False, .060, True, True, True, 20, True), 0.)


    def test_physical_locking_paths(self):
        from task import LATCH, SECONDARY, SOCKET
        latch_gate, rail_gate = LATCH.boxes[-1], SECONDARY.boxes[0]
        lug, shoulder = LATCH.boxes[0], SOCKET.boxes[4]

        def bounds(shape, index):
            center, size, _ = shape
            return center[index] - size[index] / 2, center[index] + size[index] / 2

        # Half travel leaves the rail's blocking face in the latch footprint.
        self.assertGreater(bounds(rail_gate, 1)[1] - .006, bounds(latch_gate, 1)[0])
        self.assertLess(bounds(rail_gate, 1)[1] - SECONDARY.travel, bounds(latch_gate, 1)[0])
        self.assertGreater(bounds(lug, 0)[1], bounds(shoulder, 0)[0])
        self.assertLess(bounds(lug, 0)[1] - .004, bounds(shoulder, 0)[0])
