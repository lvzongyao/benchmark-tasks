import unittest

import numpy as np

from scoring import FLOOR, SLEEVE_Z, TRAY_Z
from task import SLEEVE


class SleeveWindow(unittest.TestCase):
    def test_physical_pane_closes_the_roof_and_preserves_tray_headroom(self):
        ((center, size, _),) = SLEEVE.optical_panels
        low, high = np.asarray(center)-np.asarray(size)/2, np.asarray(center)+np.asarray(size)/2
        np.testing.assert_allclose(low, [-.043, -.071, SLEEVE_Z-FLOOR])
        np.testing.assert_allclose(high, [.043, .084, SLEEVE_Z])
        self.assertAlmostEqual(low[2]-(FLOOR+TRAY_Z), .002)
