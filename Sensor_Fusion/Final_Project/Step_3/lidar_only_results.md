# Step 3 — Lidar-Only Multi-Object Tracking Results

## Overview

In Step 3, a lidar-only multi-object tracking pipeline was implemented using:

- Extended Kalman Filter (EKF)
- Track management
- Nearest Neighbor data association

At this stage, only lidar measurements were used for object tracking. Camera fusion was not included yet.

The objective was to:
- predict object motion,
- associate lidar detections with existing tracks,
- update tracks over time,
- remove unstable tracks,
- evaluate tracking quality using RMSE.

---

# Tracking Components

## 1. Extended Kalman Filter (EKF)

The EKF was used to estimate object states over time.

Implemented components:
- state prediction using a constant velocity motion model,
- covariance prediction,
- Kalman gain computation,
- state update using lidar measurements.

The state vector used was:

```text
[x, y, z, vx, vy, vz]
```

Lidar measurements directly updated:
- x position,
- y position,
- z position.

---

## 2. Track Management

Track management logic handled:
- initialization of new tracks,
- score updates,
- confirmation of stable tracks,
- deletion of unstable tracks.

Implemented logic:
- tracks initialized from unassigned lidar detections,
- tentative tracks promoted to confirmed after repeated successful updates,
- tracks deleted if uncertainty became too large,
- tracks deleted if score dropped below threshold.

---

## 3. Data Association

Nearest Neighbor association was implemented using:
- Mahalanobis distance,
- association matrix,
- gating logic.

The following operations were implemented:
- association matrix initialization,
- minimum distance selection,
- row and column removal after successful matching,
- gating threshold checks.

---

# Lidar-Only Results

The lidar-only tracker successfully maintained multiple confirmed tracks across the sequence.

## RMSE Evaluation

The final RMSE plot showed several stable tracks with low localization error.

| Track ID | Mean RMSE |
|----------|------------|
| Track 0  | 0.15 m |
| Track 1  | 0.12 m |
| Track 10 | 0.20 m |

These results satisfy the project requirement that at least two long-duration tracks maintain RMSE below 0.25 m.

---

# Observations

## Strong Tracking Performance

Lidar measurements provided accurate geometric localization, allowing the EKF to maintain stable object trajectories throughout the sequence.

## Stable Long-Term Tracks

Tracks 0 and 1 remained stable over long durations with low RMSE values.

## High RMSE Outlier

Track 3 showed a significantly larger RMSE value:

```text
6.74 m
```

This likely occurred because of:
- track fragmentation,
- temporary association mismatch,
- stale track persistence,
- noisy detections near field-of-view boundaries.

Despite this outlier, the primary long-duration tracks remained stable and satisfied the required threshold.

---

# Challenges Faced

The most difficult part of the project was tuning:
- association gating,
- deletion thresholds,
- process noise parameters.

Incorrect tuning occasionally caused:
- unstable tracks,
- identity switching,
- large RMSE spikes.

Debugging required:
- visual inspection of trajectories,
- track lifecycle analysis,
- repeated RMSE evaluation.

---

# Conclusion

The lidar-only tracking pipeline successfully demonstrated:
- stable object tracking,
- accurate EKF estimation,
- effective nearest-neighbor association,
- robust track management.

The tracker achieved low RMSE values on multiple long-duration tracks and provided a strong foundation for Step 4 sensor fusion.
