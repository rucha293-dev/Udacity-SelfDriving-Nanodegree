# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params


class Filter:
    '''Kalman filter class'''

    def __init__(self):
        self.dim_state = params.dim_state  # load dim_state from params

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############
        dt = params.dt  # load dt from params
        # Constant velocity model: state vector is [x, y, z, vx, vy, vz]
        return np.matrix([
            [1, 0, 0, dt, 0,  0 ],
            [0, 1, 0, 0,  dt, 0 ],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ]
        ])
        ############
        # END student code
        ############

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############
        dt = params.dt  # load dt from params
        q  = params.q   # load q from params

        # Derived terms for the continuous white noise acceleration model
        q3 = (dt**3) / 3 * q
        q2 = (dt**2) / 2 * q
        q1 = dt * q

        return np.matrix([
            [q3, 0,  0,  q2, 0,  0 ],
            [0,  q3, 0,  0,  q2, 0 ],
            [0,  0,  q3, 0,  0,  q2],
            [q2, 0,  0,  q1, 0,  0 ],
            [0,  q2, 0,  0,  q1, 0 ],
            [0,  0,  q2, 0,  0,  q1]
        ])
        ############
        # END student code
        ############

    def predict(self, track):
        ############
        # Step 1: predict state x and estimation error covariance P to next timestep
        ############
        F = self.F()
        Q = self.Q()

        x = F * track.x            # predicted state
        P = F * track.P * F.T + Q  # predicted covariance

        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############

    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement
        ############
        H     = meas.sensor.get_H(track.x)          # measurement matrix (linearised if needed)
        gamma = self.gamma(track, meas)              # residual
        S     = self.S(track, meas, H)               # residual covariance
        K     = track.P * H.T * np.linalg.inv(S)    # Kalman gain
        x     = track.x + K * gamma
        I     = np.identity(self.dim_state)          # use dim_state loaded from params
        P     = (I - K * H) * track.P

        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############
        track.update_attributes(meas)

    def gamma(self, track, meas):
        ############
        # Step 1: calculate and return residual gamma
        ############
        hx = meas.sensor.get_hx(track.x)
        return meas.z - hx
        ############
        # END student code
        ############

    def S(self, track, meas, H):
        ############
        # Step 1: calculate and return covariance of residual S
        ############
        return H * track.P * H.T + meas.R
        ############
        # END student code
        ############
