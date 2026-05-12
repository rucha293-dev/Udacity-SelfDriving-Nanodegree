# Multi-Sensor 3D Object Tracking

## Project Overview

This project implements a multi-object tracking pipeline using lidar and camera data. The objective is to estimate and maintain the state of multiple objects over time using probabilistic estimation and sensor fusion techniques.

The tracking framework integrates:
- Extended Kalman Filtering (EKF)
- Data Association
- Track Management
- Camera-Lidar Sensor Fusion

The project is based on the Sensor Fusion module from the Udacity Self-Driving Car Engineer Nanodegree.

---

# Project Structure

```text
Sensor_Fusion/
│
├── Code/
│   ├── association.py
│   ├── filter.py
│   ├── helpers.py
│   ├── loop_over_dataset.py
│   ├── measurements.py
│   ├── objdet_detect.py
│   ├── objdet_eval.py
│   ├── objdet_pcl.py
│   ├── params.py
│   └── trackmanagement.py
│
├── Final_Project/
│
├── Step_1/
│   ├── rmse.png
│   └── tracking_step1.png
│
├── Step_2/
│   └── rmsetask2.png
│
├── Step_3/
│   ├── lidar_only_results.md
│   └── rmse.png
│
├── Step_4/
│   ├── final_results.md
│   ├── rmse_step4.png
│   └── tracking_step4.png
│
├── Results/
│   └── tracking_final.mp4
│
├── README.md
├── requirements.txt
│
└── Mid-Term Project - 3D Object Detection/
```

---

# Environment Used

This project was developed and tested using:

- Ubuntu 20.04
- Python 3.8+
- Udacity Sensor Fusion workspace
- Docker-based classroom environment

---

# Required Python Packages

Install dependencies using:

```bash
pip install -r requirements.txt
```

Contents of:

```text
requirements.txt
```

```text
numpy
opencv-python
protobuf
easydict
torch
pillow
matplotlib
wxpython
shapely
tqdm
open3d
```

---

# Dataset Location

Place the Waymo dataset sequence inside:

```text
Sensor_Fusion/dataset/
```

Example:

```text
Sensor_Fusion/dataset/training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord
```

---

# Running the Tracking Pipeline

Navigate to:

```bash
cd Sensor_Fusion/Code
```

Run:

```bash
python loop_over_dataset.py
```

---

# Flags Used for Step 1–4

## Step 1 — EKF Prediction / Update

Implemented in:

```text
filter.py
```

Functions implemented:
- `predict()`
- `update()`
- `gamma()`
- `S()`
- `F()`
- `Q()`

The prediction step uses a constant velocity motion model in 3D space.

The update step incorporates:
- lidar measurements,
- nonlinear camera measurements.

---

## Step 2 — Track Management

Implemented in:

```text
trackmanagement.py
```

Implemented features:
- initialization of tracks from lidar measurements,
- score update logic,
- tentative and confirmed track states,
- deletion logic based on score and covariance.

Track initialization transforms measurements from sensor coordinates into vehicle coordinates using the sensor transformation matrix.

---

## Step 3 — Data Association and Lidar-Only Tracking

Implemented in:

```text
association.py
```

Implemented components:
- Mahalanobis distance,
- gating,
- association matrix,
- nearest-neighbor matching.

The closest valid measurement-track pair is selected iteratively until no valid associations remain.

### Lidar-Only Tracking

Inside:

```text
measurements.py
```

measurements were restricted to lidar only inside:

```python
generate_measurement()
```

Example:

```python
if self.name == 'lidar':

    meas = Measurement(num_frame, z, self)

    meas_list.append(meas)
```

---

## Step 4 — Camera + Lidar Fusion

Camera fusion was implemented in:

```text
measurements.py
```

Implemented functions:
- `in_fov()`
- `get_hx()`
- `get_H()`
- camera measurement initialization
- camera measurement generation

Camera measurements were integrated into the EKF using nonlinear projection equations and Jacobian linearization.

To enable fusion:

```python
meas = Measurement(num_frame, z, self)

meas_list.append(meas)
```

inside:

```python
generate_measurement()
```

---

# Tracking Visualization and Movie Generation

Inside:

```text
loop_over_dataset.py
```

set:

```python
exec_visualization = [
    'show_tracks',
    'make_tracking_movie'
]
```

Then rerun:

```bash
python loop_over_dataset.py
```

---

# Expected Output Paths

## RMSE Plots

```text
Step_3/rmse.png
Step_4/rmse_step4.png
```

---

## Tracking Movie

```text
Results/tracking_final.mp4
```

---

## Tracking Frames

```text
results/tracking000.png
results/tracking001.png
...
```

---

# Writeup: Track 3D-Objects Over Time

## 1. Recap of the Four Tracking Steps

### Step 1 — Extended Kalman Filter

The Extended Kalman Filter (EKF) was implemented in:

```text
filter.py
```

The following functions were implemented:
- `predict()`
- `update()`
- `gamma()`
- `S()`
- `F()`
- `Q()`

The prediction step used a constant velocity motion model in 3D space.

The update step incorporated:
- lidar measurements,
- nonlinear camera measurements.

The state vector used was:

```text
[x, y, z, vx, vy, vz]
```

---

### Step 2 — Track Management

Track management was implemented in:

```text
trackmanagement.py
```

Implemented features:
- initialization of tracks from lidar measurements,
- score updates,
- tentative and confirmed track states,
- deletion logic based on score and covariance.

Track initialization transformed measurements from sensor coordinates into vehicle coordinates using the sensor transformation matrix.

---

### Step 3 — Data Association

Data association was implemented in:

```text
association.py
```

Implemented components:
- Mahalanobis distance,
- gating,
- association matrix,
- nearest-neighbor matching.

The closest valid measurement-track pair was selected iteratively until no valid associations remained.

---

### Step 4 — Camera-Lidar Fusion

Camera fusion was implemented in:

```text
measurements.py
```

Implemented functions:
- `in_fov()`
- `get_hx()`
- `get_H()`
- camera measurement initialization
- camera measurement generation

Camera measurements were integrated into the EKF using nonlinear projection equations and Jacobian linearization.

---

# Results Achieved

## Step 3 — Lidar-Only Results

| Track ID | Mean RMSE |
|----------|------------|
| Track 0  | 0.15 m |
| Track 1  | 0.12 m |
| Track 10 | 0.20 m |

The lidar-only tracker maintained multiple long-duration tracks below the required RMSE threshold of:

```text
0.25 m
```

---

## Step 4 — Camera + Lidar Fusion Results

| Track ID | Mean RMSE |
|----------|------------|
| Track 0  | 0.17 m |
| Track 1  | 0.10 m |
| Track 34 | 0.11 m |

The fused tracker successfully maintained stable confirmed tracks with low localization error.

---

# Most Difficult Part of the Project

The most difficult part of the project was debugging and tuning the interaction between:
- data association,
- track management,
- sensor fusion.

Small implementation errors in:
- Mahalanobis distance computation,
- gating thresholds,
- Jacobian calculations,
- covariance updates,

could destabilize the entire tracking pipeline and lead to:
- unstable tracks,
- identity switches,
- large RMSE spikes.

Another challenging aspect was integrating nonlinear camera measurements into the EKF framework. Camera updates required:
- coordinate transformations,
- nonlinear projection functions,
- Jacobian computation,
- division-by-zero handling.

Debugging required repeatedly analyzing:
- RMSE plots,
- track lifetimes,
- covariance growth,
- visualization outputs.

---

# 2. Benefits of Camera-Lidar Fusion

## Theoretical Benefits

Lidar provides:
- accurate geometric localization,
- depth measurements,
- stable 3D positioning.

Camera provides:
- contextual information,
- additional geometric constraints,
- image-space observations.

Combining both sensors improves overall tracking robustness.

---

## Observed Results

In the project:
- lidar-only tracking already achieved strong localization accuracy,
- camera fusion improved tracking consistency and robustness,
- fused tracking handled partial observations more reliably.

Although RMSE improvements were modest, fusion improved track stability in challenging scenarios.

---

# 3. Real-World Challenges for Sensor Fusion

Real-world sensor fusion systems face several challenges:

- sensor noise,
- calibration drift,
- synchronization issues,
- occlusions,
- adverse weather,
- changing lighting conditions.

Some of these challenges appeared during the project as:
- unstable tracks,
- association mismatches,
- RMSE spikes,
- sensitivity to parameter tuning.

---

# 4. Future Improvements

Potential future improvements include:

- Hungarian Algorithm for global association,
- Unscented Kalman Filter (UKF),
- adaptive gating thresholds,
- improved motion models,
- deep learning-based tracking,
- monocular depth estimation,
- multi-hypothesis tracking.

---

# Technologies Used

- Python
- NumPy
- OpenCV
- Extended Kalman Filter
- Sensor Fusion
- Multi-Object Tracking

---

# Conclusion

This project demonstrates how:
- Extended Kalman Filtering,
- Track Management,
- Data Association,
- Multi-Sensor Fusion

can be combined to achieve robust multi-object tracking for autonomous driving systems.

The integration of lidar and camera measurements improves tracking reliability and highlights the importance of sensor fusion in autonomous vehicle perception systems.
