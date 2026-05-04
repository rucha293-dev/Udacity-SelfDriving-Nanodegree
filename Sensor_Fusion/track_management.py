# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import os
import sys

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 


# ---------------------------
# Track Class
# ---------------------------
class Track:
    '''Track class with state and covariance'''
    def __init__(self, meas, id):
        print('creating track no.', id)

        M_rot = meas.sensor.sens_to_veh[0:3, 0:3]

        # Transform measurement position to vehicle coordinates
        pos_sens = np.matrix([
            [meas.z[0, 0]],
            [meas.z[1, 0]],
            [meas.z[2, 0]],
            [1.0]
        ])

        pos_veh = meas.sensor.sens_to_veh * pos_sens

        # State vector [x, y, z, vx, vy, vz]
        self.x = np.matrix([
            [pos_veh[0, 0]],
            [pos_veh[1, 0]],
            [pos_veh[2, 0]],
            [0.],
            [0.],
            [0.]
        ])

        # Estimation error covariance matrix P
        self.P = np.matrix([
            [params.sigma_lidar_x**2, 0, 0, 0, 0, 0],
            [0, params.sigma_lidar_y**2, 0, 0, 0, 0],
            [0, 0, params.sigma_lidar_z**2, 0, 0, 0],
            [0, 0, 0, params.sigma_p44**2, 0, 0],
            [0, 0, 0, 0, params.sigma_p55**2, 0],
            [0, 0, 0, 0, 0, params.sigma_p66**2]
        ])

        # Track state and score
        self.state = 'tentative'
        self.score = 1.0 / params.window  # initial score

        # Track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height

        self.yaw = np.arccos(
            M_rot[0, 0] * np.cos(meas.yaw) +
            M_rot[0, 1] * np.sin(meas.yaw)
        )

        self.t = meas.t

    def set_x(self, x):
        self.x = x

    def set_P(self, P):
        self.P = P

    def set_t(self, t):
        self.t = t

    def update_attributes(self, meas):
        if meas.sensor.name == 'lidar':
            c = params.weight_dim

            self.width  = c * meas.width  + (1 - c) * self.width
            self.length = c * meas.length + (1 - c) * self.length
            self.height = c * meas.height + (1 - c) * self.height

            M_rot = meas.sensor.sens_to_veh

            self.yaw = np.arccos(
                M_rot[0, 0] * np.cos(meas.yaw) +
                M_rot[0, 1] * np.sin(meas.yaw)
            )


# ---------------------------
# Track Management
# ---------------------------
class Trackmanagement:
    '''Track manager class'''
    def __init__(self):
        self.N = 0
        self.track_list = []
        self.last_id = -1
        self.result_list = []

    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):

        # Decrease score for unassigned tracks that are in sensor FOV
        for i in unassigned_tracks:
            track = self.track_list[i]
            if len(meas_list) > 0:
                sensor = meas_list[0].sensor
                if sensor.in_fov(track.x):
                    track.score -= 1.0 / params.window

        # Delete tracks that are below threshold or have high uncertainty
        for track in self.track_list[:]:
            if track.state == 'confirmed' and track.score < params.delete_threshold:
                self.delete_track(track)
            elif track.state == 'tentative' and track.score < 1.0 / params.window:
                self.delete_track(track)
            elif track.P[0, 0] > params.max_P or track.P[1, 1] > params.max_P:
                self.delete_track(track)

        # Initialize new tracks from unassigned lidar measurements only
        for j in unassigned_meas:
            if meas_list[j].sensor.name == 'lidar':
                self.init_track(meas_list[j])

    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)

    def handle_updated_track(self, track, meas):

        # Increase score for lidar updates only
        if meas.sensor.name == 'lidar':
            track.score = min(track.score + 1.0 / params.window, 1.0)

        # State transition based on score
        if track.score >= params.confirmed_threshold:
            track.state = 'confirmed'
        elif track.state != 'confirmed':
            track.state = 'tentative'
