#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 3 - Part I

PKG = "shutter_track_target_public_tests"
NAME = 'test_kalman_filter'

import sys
import unittest

import rospy
import rostest
from std_msgs.msg import Header
from shutter_track_target.msg import Observation

from utils import compute_import_path

import_path = compute_import_path('shutter_track_target', 'scripts')
sys.path.insert(1, import_path)
from kalman_filter import KalmanFilterNode, KF_predict_step, KF_measurement_update_step

class TestKalmanFilter(unittest.TestCase):
    """
    Public tests for detect_visual_target.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestKalmanFilter, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)

        self.node = KalmanFilterNode.__new__(KalmanFilterNode)

    def create_obs_msg(self):
        x = 1
        y = 8
        obs_msg = Observation()
        header = Header()
        header.stamp = rospy.Time.now()
        obs_msg.header = header
        obs_msg.x = x
        obs_msg.y = y
        return obs_msg

    def initialize_node(self):
        self.node.assemble_A_matrix(0.1)
        self.node.assemble_C_matrix()
        self.node.initialize_process_covariance()
        self.node.initialize_measurement_covariance()

    def test_a_assemble_A_matrix(self):
        """
        Callback to that the A matrix has the right dimensions
        """
        self.node.assemble_A_matrix(0.1)
        A = self.node.A
        self.assertTrue(A.shape==(6,6), "A is not the correct shape")

    def test_b_assemble_C_matrix(self):
        """
        Check that the C matrix has the right dimensions
        """
        self.node.assemble_C_matrix()
        C = self.node.C
        self.assertTrue(C.shape==(2,6), "C is not the correct shape")

    def test_c_initialize_process_covariance(self):
        """
        Check that the R matrix has the right dimensions
        """
        self.node.initialize_process_covariance()
        R = self.node.R
        self.assertTrue(R.shape==(6,6), "R is not the correct shape")

    def test_d_initialize_measurement_covariance(self):
        """
        Check that the Q matrix has the right dimensions
        """
        self.node.initialize_measurement_covariance()
        Q = self.node.Q
        self.assertTrue(Q.shape==(2,2), "Q is not the correct shape")

    def test_e_assemble_observation_vector(self):
        """
        Check that the observation vector is 2D
        """
        obs_msg = self.create_obs_msg()
        z = self.node.assemble_observation_vector(obs_msg)
        self.assertTrue(z.shape==(2,1), "z is not the right shape.")

    def test_f_initialize_mu_and_sigma(self):
        """
        Check that the filter is initialized
        """
        obs_msg = self.create_obs_msg()
        self.node.initialize_mu_and_sigma(obs_msg)
        
        mu = self.node.mu
        self.assertTrue(mu.shape==(6,1), "mu is not the correct shape")

        Sigma = self.node.Sigma
        self.assertTrue(Sigma.shape==(6,6), "Sigma is not the correct shape")

    def test_g_KF_predict_step(self):
        """
        Check that prediction step function is returning some value
        """
        self.initialize_node()
        obs_msg = self.create_obs_msg()
        self.node.initialize_mu_and_sigma(obs_msg)

        mu, Sigma = KF_predict_step(self.node.mu, self.node.Sigma, self.node.A, self.node.R)

        self.assertIsNotNone(mu, 'mu is None')
        self.assertIsNotNone(Sigma, 'Sigma is not None')

    def test_h_KF_measurement_update_step(self):
        """
        Check that the measurement update function is returning some value
        """
        self.initialize_node()
        obs_msg = self.create_obs_msg()
        self.node.initialize_mu_and_sigma(obs_msg)
        z = self.node.assemble_observation_vector(obs_msg)

        pred_mu, pred_Sigma = KF_predict_step(self.node.mu, self.node.Sigma, self.node.A, self.node.R)
        new_mu, new_Sigma = KF_measurement_update_step(pred_mu, pred_Sigma, z, self.node.C, self.node.Q)

        self.assertIsNotNone(new_mu, 'mu is None')
        self.assertIsNotNone(new_Sigma, 'Sigma is not None')

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestKalmanFilter, sys.argv)
