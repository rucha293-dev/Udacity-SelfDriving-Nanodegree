# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # Step 2: initialization of x and P based on unassigned measurement
        # transformed from sensor to vehicle coordinates
        ############

        # Transform measurement position from sensor to vehicle coordinates
        # sens_to_veh is a 4x4 homogeneous transform matrix
        pos_sens = np.matrix([[meas.z[0, 0]],
                              [meas.z[1, 0]],
                              [meas.z[2, 0]],
                              [1.0]])
        pos_veh = meas.sensor.sens_to_veh * pos_sens  # 4x1 in vehicle coords

        # Initialize state vector [x, y, z, vx, vy, vz]
        # Velocities are unknown at initialization, so set to 0
        self.x = np.matrix([[pos_veh[0, 0]],
                             [pos_veh[1, 0]],
                             [pos_veh[2, 0]],
                             [0.],
                             [0.],
                             [0.]])

        # Initialize estimation error covariance P
        # Position variances come from sensor measurement noise (sigma^2)
        # Velocity variances use large initial values from params (high uncertainty)
        sigma_x = params.sigma_lidar_x
        sigma_y = params.sigma_lidar_y
        sigma_z = params.sigma_lidar_z

        self.P = np.matrix([
            [sigma_x**2, 0,          0,          0,                  0,                  0               ],
            [0,          sigma_y**2,  0,          0,                  0,                  0               ],
            [0,          0,          sigma_z**2,  0,                  0,                  0               ],
            [0,          0,          0,          params.sigma_p44**2, 0,                  0               ],
            [0,          0,          0,          0,                  params.sigma_p55**2, 0               ],
            [0,          0,          0,          0,                  0,                  params.sigma_p66**2]
        ])

        # New track starts as 'tentative' with a low initial score
        # Score is expressed as a fraction: detections / window
        self.state = 'tentative'
        self.score = 1. / params.window
        
        ############
        # END student code
        ############ 
               
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t  
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################        

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):  
        ############
        # Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big
        ############
        
        # decrease score for unassigned tracks
        for i in unassigned_tracks:
            track = self.track_list[i]
            # only penalize if the track was visible to the sensor this frame
            if meas_list:
                if meas_list[0].sensor.in_fov(track.x):
                    # Track was in FOV but got no measurement → lower score
                    track.score -= 1. / params.window

        # delete tracks that are too uncertain or have too low a score
        for track in self.track_list[:]:  # iterate over a copy so deletion is safe
            # delete confirmed tracks whose score drops below delete_threshold
            if track.state == 'confirmed' and track.score < params.delete_threshold:
                self.delete_track(track)
            # delete tentative tracks whose score is too low
            elif track.state == 'tentative' and track.score < 1. / params.window:
                self.delete_track(track)
            # delete any track whose position covariance has grown too large
            elif track.P[0, 0] > params.max_P or track.P[1, 1] > params.max_P:
                self.delete_track(track)

        ############
        # END student code
        ############ 
            
        # initialize new track with unassigned measurement
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
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
        
    def handle_updated_track(self, track):      
        ############
        # Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############

        # Increase score (cap at 1.0 to keep it a valid probability-like value)
        track.score = min(track.score + 1. / params.window, 1.0)

        # Promote to confirmed if score exceeds threshold, else keep tentative
        if track.score >= params.confirmed_threshold:
            track.state = 'confirmed'
        else:
            track.state = 'tentative'
        
        ############
        # END student code
        ############
