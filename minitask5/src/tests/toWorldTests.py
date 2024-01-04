#!/usr/bin/env python

import sys
sys.path.append("..")
import unittest
from common.OccupiedGrid import OccupiedGrid

class toWorldTests(unittest.TestCase):

    def testValid1(self):
        testx, testy = OccupiedGrid.to_world(OccupiedGrid(), 9, 9, (-5, -5), (20,20), 1.0)
        self.assertEqual((testx, testy), (4.5, 4.5))


if __name__ == '__main__':
    unittest.main()