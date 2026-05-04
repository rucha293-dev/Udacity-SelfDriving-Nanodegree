# Multi-Sensor 3D Object Tracking

## Project Overview

This project implements a multi-object tracking pipeline using lidar and camera data. The objective is to estimate and maintain the state of multiple objects over time by combining measurements from different sensors using probabilistic estimation techniques.

The system integrates detection, state estimation, data association, and track management into a unified framework.

---

## Tracking Pipeline

### 1. Extended Kalman Filter (EKF)

The Extended Kalman Filter is used to estimate object states over time.

- The state vector includes position and velocity in 3D space.
- The prediction step uses a constant velocity motion model.
- The update step incorporates measurements from:
  - Lidar (linear measurement model)
  - Camera (nonlinear projection model using a Jacobian)

---

### 2. Track Management

Track management is responsible for maintaining the lifecycle of tracked objects.

- New tracks are initialized from unassigned measurements.
- Tracks are confirmed after consistent observations across multiple frames.
- Tracks are deleted if they are not updated for a predefined number of frames.

A score-based system is used to determine track validity.

---

### 3. Data Association

Data association assigns incoming measurements to existing tracks.

- Nearest Neighbor association is used.
- Mahalanobis distance is applied as the similarity metric.
- Gating is implemented to reject unlikely associations.

---

### 4. Camera-Lidar Sensor Fusion

Sensor fusion combines the strengths of lidar and camera measurements.

- Lidar provides accurate 3D position estimates.
- Camera provides additional contextual and geometric constraints.

Camera measurements are integrated using nonlinear updates within the EKF framework.

---

## Results

- Successful tracking of multiple objects across frames
- Consistent track identities over time
- Reduction in false associations through gating
- Improved tracking robustness when using both sensors

Lidar provided strong geometric accuracy, while camera measurements contributed to improved consistency in certain scenarios.

---

## Challenges

### Data Association

Small errors in association can lead to incorrect track assignments and identity switches. Proper tuning of gating thresholds is critical.

### Track Management

Balancing track persistence and deletion is challenging, particularly in the presence of missed detections.

### Sensor Fusion

Accurate calibration and synchronization between sensors are essential. Errors in these areas degrade performance.

---

## Benefits of Sensor Fusion

### Theoretical Perspective

- Lidar provides precise geometric information (depth and position).
- Camera provides semantic and contextual information.

Combining both sensors improves overall perception capability.

### Observed Results

- Increased tracking stability
- Reduced ambiguity in detections
- Improved handling of partial observations

---

## Real-World Challenges

Sensor fusion systems face several challenges in real-world applications:

- Sensor noise affecting measurement accuracy
- Calibration errors between sensors
- Time synchronization issues
- Occlusions in dynamic environments
- Adverse weather and lighting conditions

In this project, some of these challenges were observed as missed detections, association mismatches, and sensitivity to parameter tuning.

---

## Future Improvements

- Replace Nearest Neighbor with the Hungarian Algorithm for global association
- Use an Unscented Kalman Filter for improved nonlinear estimation
- Implement more advanced motion models (e.g., constant acceleration)
- Introduce adaptive thresholds in track management
- Incorporate confidence scores from deep learning-based detectors

---

## Technologies Used

- Python
- NumPy
- OpenCV
- Kalman filtering
- Sensor fusion techniques

---






---

## Conclusion

This project demonstrates how Extended Kalman Filtering, data association, track management, and sensor fusion can be combined to achieve reliable multi-object tracking.

The integration of lidar and camera data improves robustness and performance compared to single-sensor approaches, highlighting the importance of sensor fusion in autonomous driving systems.
