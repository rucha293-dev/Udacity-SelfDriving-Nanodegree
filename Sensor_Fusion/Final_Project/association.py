# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor
#                        association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params


class Association:
    '''Data association class with single nearest neighbor association
       and gating based on Mahalanobis distance'''

    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []

    def associate(self, track_list, meas_list, KF):
        ############
        # Step 3: association:
        # - replace association_matrix with the actual association matrix based on
        #   Mahalanobis distance for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############

        N = len(track_list)
        M = len(meas_list)

        # initialise all tracks and measurements as unassigned
        self.unassigned_tracks = list(range(N))
        self.unassigned_meas   = list(range(M))

        # build N x M association matrix filled with infinity
        self.association_matrix = np.inf * np.ones((N, M))

        for i, track in enumerate(track_list):
            for j, meas in enumerate(meas_list):
                # compute Mahalanobis distance
                dist = self.MHD(track, meas, KF)
                # only fill entry if measurement lies inside the gate
                if self.gating(dist, meas.sensor):
                    self.association_matrix[i, j] = dist

        self.association_matrix = np.matrix(self.association_matrix)

        ############
        # END student code
        ############

    def get_closest_track_and_meas(self):
        ############
        # Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned lists
        # - return this track and measurement
        ############

        A = self.association_matrix

        # if no valid association exists, return NaN
        if np.min(A) == np.inf:
            return np.nan, np.nan

        # find indices of the minimum Mahalanobis distance
        ij_min = np.unravel_index(np.argmin(A), A.shape)
        ind_track = ij_min[0]
        ind_meas  = ij_min[1]

        # map matrix indices back to the original track/measurement indices
        update_track = self.unassigned_tracks[ind_track]
        update_meas  = self.unassigned_meas[ind_meas]

        # remove the matched row and column from the association matrix
        self.association_matrix = np.delete(A, ind_track, axis=0)
        self.association_matrix = np.delete(self.association_matrix, ind_meas, axis=1)

        # remove from unassigned lists so they cannot be used again
        self.unassigned_tracks.remove(update_track)
        self.unassigned_meas.remove(update_meas)

        ############
        # END student code
        ############
        return update_track, update_meas

    def gating(self, MHD, sensor):
        ############
        # Step 3: return True if measurement lies inside gate, otherwise False
        ############

        # chi-squared threshold with sensor measurement dimension degrees of freedom
        # gating_threshold is the confidence level loaded from params
        threshold = chi2.ppf(params.gating_threshold, df=sensor.dim_meas)
        return MHD < threshold

        ############
        # END student code
        ############

    def MHD(self, track, meas, KF):
        ############
        # Step 3: calculate and return Mahalanobis distance
        ############

        H     = meas.sensor.get_H(track.x)           # linearised measurement matrix
        gamma = KF.gamma(track, meas)                 # residual z - h(x)
        S     = KF.S(track, meas, H)                  # residual covariance
        return float(gamma.T @ np.linalg.inv(S) @ gamma)

        ############
        # END student code
        ############

    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)

        # update associated tracks with measurements
        while self.association_matrix.shape[0] > 0 and self.association_matrix.shape[1] > 0:

            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]

            # check visibility, only update tracks in fov
            if not meas_list[ind_meas].sensor.in_fov(track.x):
                continue

            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])

            # update score and track state
            manager.handle_updated_track(track, meas_list[ind_meas])

            # save updated track
            manager.track_list[ind_track] = track

        # run track management
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)

        for track in manager.track_list:
            print('track', track.id, 'score =', track.score)
