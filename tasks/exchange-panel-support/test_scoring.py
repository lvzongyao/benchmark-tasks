import unittest

from scoring import Exchange


class ExchangeTests(unittest.TestCase):
    def test_drop_cannot_be_repaired_for_credit(self):
        trial = Exchange()
        self.assertEqual(trial.update(1, -.0031, .027, True, True, True, True, True), 0.)
        self.assertEqual(trial.update(2, 0., .100, True, True, True, True, True), 0.)

    def test_real_tiers(self):
        trial = Exchange()
        self.assertEqual(trial.update(1, 0., .002, True, False, False, False, True), 0.)
        self.assertEqual(trial.update(2, .001, .011, True, False, False, False, True), .25)
        self.assertEqual(trial.update(3, .002, .090, True, True, False, False, True), .50)
        self.assertEqual(trial.update(4, .001, .090, True, True, True, False, True), .75)
        self.assertEqual(trial.update(5, .001, .090, True, True, True, True, True), 1.)

    def test_wrong_block_and_lost_final_contact(self):
        trial = Exchange()
        self.assertEqual(trial.update(1, 0., 0., True, False, True, True, True), 0.)
        self.assertEqual(trial.update(2, 0., .090, True, True, True, True, True), 1.)
        self.assertEqual(trial.update(3, 0., .090, True, True, False, True, True), .50)

    def test_missing_support_during_exchange(self):
        trial = Exchange()
        self.assertEqual(trial.update(1, .001, .04, False, True, False, False, True), 0.)


if __name__ == "__main__":
    unittest.main()
