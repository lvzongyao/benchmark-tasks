import os
import unittest
from unittest.mock import patch

import numpy as np

from task import BASE, BLOCK_RESET, HULLS, OBJECTS, STOP, UPPER, demo, spacer


class FixtureFit(unittest.TestCase):
    def test_spacer_physically_supports_raised_carrier(self):
        carrier_top_of_stop = OBJECTS[1].pos[2]
        contact_center, contact_size, _ = spacer()[0]
        spacer_top = BLOCK_RESET[2]+contact_center[2]+contact_size[2]/2
        self.assertAlmostEqual(carrier_top_of_stop, BASE[2]+STOP+UPPER)
        self.assertAlmostEqual(carrier_top_of_stop, spacer_top)

    def test_spacer_and_external_handle_fit_recovery_tray(self):
        staged = HULLS[2]+[0., .018, .004]
        self.assertLess(np.max(np.abs(staged[:, 0])), .048)
        self.assertGreater(np.min(staged[:, 1]), -.103)
        self.assertLess(np.max(staged[:, 1]), .064)

    def test_unsupported_control_repeats_only_the_spacer_arm_actions(self):
        with patch.dict(os.environ, {"CARRIER_CONTROL": ""}):
            normal = demo()
        with patch.dict(os.environ, {"CARRIER_CONTROL": "unsupported"}):
            control = demo()
        self.assertEqual(set(control), {"right"})
        self.assertEqual(control["right"], normal["right"])
