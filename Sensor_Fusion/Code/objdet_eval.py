# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Evaluate performance of object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#


import numpy as np
import matplotlib.pyplot as plt
import torch
from shapely.geometry import Polygon
from operator import itemgetter
import os
import sys

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.objdet_tools as tools


def measure_detection_performance(detections, labels, labels_valid, min_iou=0.5):
    
    true_positives = 0
    center_devs = []
    ious = []

    for label, valid in zip(labels, labels_valid):
        matches_lab_det = []

        if valid:

            ####### ID_S4_EX1 START #######     
            #######
            print("student task ID_S4_EX1 ")

            ## step 1 : extract the four corners of the current label bounding-box
            label_corners = tools.compute_box_corners(
                label.box.center_x,
                label.box.center_y,
                label.box.width,
                label.box.length,
                label.box.heading
            )
            label_poly = Polygon(label_corners)

            ## step 2 : loop over all detected objects
            for det in detections:

                ## step 3 : extract the four corners of the current detection
                det_corners = tools.compute_box_corners(
                    det[1],  # x
                    det[2],  # y
                    det[5],  # width
                    det[6],  # length
                    det[7]   # heading
                )
                det_poly = Polygon(det_corners)

                ## step 4 : compute the center distance between label and detection
                dist_x = label.box.center_x - det[1]
                dist_y = label.box.center_y - det[2]
                dist_z = label.box.center_z - det[3]

                ## step 5 : compute the intersection over union (IOU)
                if label_poly.is_valid and det_poly.is_valid:
                    intersection = label_poly.intersection(det_poly).area
                    union = label_poly.union(det_poly).area
                    iou = intersection / union if union > 0 else 0
                else:
                    iou = 0

                ## step 6 : store matches if IoU above threshold
                if iou >= min_iou:
                    matches_lab_det.append([iou, dist_x, dist_y, dist_z])

            #######
            ####### ID_S4_EX1 END #######     

        # find best match
        if matches_lab_det:
            best_match = max(matches_lab_det, key=itemgetter(0))
            true_positives += 1
            ious.append(best_match[0])
            center_devs.append(best_match[1:])

    ####### ID_S4_EX2 START #######     
    #######
    print("student task ID_S4_EX2")
    
    ## step 1 : compute total positives
    all_positives = sum(labels_valid)

    ## step 2 : compute false negatives
    false_negatives = all_positives - true_positives

    ## step 3 : compute false positives
    false_positives = len(detections) - true_positives
    
    #######
    ####### ID_S4_EX2 END #######     

    pos_negs = [all_positives, true_positives, false_negatives, false_positives]
    det_performance = [ious, center_devs, pos_negs]
    
    return det_performance


def compute_performance_stats(det_performance_all):

    ious = []
    center_devs = []
    pos_negs = []

    for item in det_performance_all:
        ious.append(item[0])
        center_devs.append(item[1])
        pos_negs.append(item[2])

    ####### ID_S4_EX3 START #######     
    #######    
    print('student task ID_S4_EX3')

    ## step 1 : extract totals
    all_positives = sum([pn[0] for pn in pos_negs])
    true_positives = sum([pn[1] for pn in pos_negs])
    false_negatives = sum([pn[2] for pn in pos_negs])
    false_positives = sum([pn[3] for pn in pos_negs])

    ## step 2 : compute precision
    precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0.0

    ## step 3 : compute recall 
    recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0.0

    #######    
    ####### ID_S4_EX3 END #######     

    print('precision = ' + str(precision) + ", recall = " + str(recall))

    ious_all = [element for tupl in ious for element in tupl]

    devs_x_all = []
    devs_y_all = []
    devs_z_all = []

    for tuple_ in center_devs:
        for elem in tuple_:
            dev_x, dev_y, dev_z = elem
            devs_x_all.append(dev_x)
            devs_y_all.append(dev_y)
            devs_z_all.append(dev_z)

    # statistics
    data = [precision, recall, ious_all, devs_x_all, devs_y_all, devs_z_all]
    titles = [
        'detection precision',
        'detection recall',
        'intersection over union',
        'position errors in X',
        'position errors in Y',
        'position error in Z'
    ]

    f, a = plt.subplots(2, 3)
    a = a.ravel()

    for idx, ax in enumerate(a):
        ax.hist(data[idx], bins=20)
        ax.set_title(titles[idx])

    plt.tight_layout()
    plt.show()

