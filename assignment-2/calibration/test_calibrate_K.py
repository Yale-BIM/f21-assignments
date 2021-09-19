#!/usr/bin/env python3
# Public tests for 559 Assignment 2 - Part V

import unittest
import numpy as np

from calibrate_K import compute_K

class TestCalibrateK(unittest.TestCase):
    """
    Public tests for calibrate_K.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestCalibrateK, self).__init__(*args)

        self.input_file = "correspondences.txt"

    def test_compute_K(self):
        """
        Check calculation of K
        """
        K, error = compute_K(self.input_file)
        print('K: {}'.format(K.shape))
        self.assertTrue(K.shape == (3,3), "Shape of K is not (3,3)")
        self.assertFalse(np.isinf(error), "Error is not finite.")

        print("Verified that 'compute_K' returns a 3x3 numpy matrix and a finite error")

