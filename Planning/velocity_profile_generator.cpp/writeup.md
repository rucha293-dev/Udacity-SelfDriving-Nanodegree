
# Velocity Profile Generator

## Overview

This file implements the velocity planning module for autonomous driving.

The velocity planner generates speed trajectories for different driving behaviors:

- nominal lane following
- deceleration to stop
- lead vehicle following

The planner computes smooth velocity profiles along a previously generated spiral path.

---

# Velocity Planning Pipeline

```text
Behavior Planner State
          ↓
Generate Spiral Path
          ↓
Generate Velocity Profile
          ↓
Assign Speed at Each Path Point
          ↓
Output Trajectory
```

---

# Supported Maneuvers

## 1. NOMINAL

Normal lane-following behavior.

Vehicle accelerates or decelerates toward desired speed.

---

## 2. DECEL_TO_STOP

Smoothly slows the vehicle to zero speed at a stop line.

---

## 3. FOLLOW_VEHICLE

Maintains safe distance from a lead vehicle.

(Currently placeholder implementation.)

---

# Trajectory Generation

## Function

```cpp
generate_trajectory(...)
```

## Purpose

Selects the correct velocity profile based on the current maneuver state.

---

# Maneuver Selection

## Implementation

```cpp
if (maneuver == DECEL_TO_STOP)
```

```cpp
else if (maneuver == FOLLOW_VEHICLE)
```

```cpp
else
```

## Purpose

Chooses the appropriate velocity generation strategy.

---

# 1. Deceleration to Stop Profile

## Function

```cpp
decelerate_trajectory(...)
```

## Purpose

Generates a smooth trapezoidal velocity profile that brings the vehicle to a complete stop.

---

# Deceleration Profile Structure

The profile consists of:

```text
Initial Speed
      ↓
Smooth Deceleration
      ↓
Slow Constant Speed
      ↓
Final Braking
      ↓
Zero Velocity
```

---

# Deceleration Distance

## Implementation

```cpp
auto decel_distance =
    calc_distance(
        start_speed,
        _slow_speed,
        -_a_max);
```

## Purpose

Computes the distance required to slow from:

```text
start_speed → slow_speed
```

---

# Brake Distance

## Implementation

```cpp
auto brake_distance =
    calc_distance(
        _slow_speed,
        0,
        -_a_max);
```

## Purpose

Computes stopping distance from slow speed to zero.

---

# Distance Equation

## Implementation

```cpp
d = (v_f^2 - v_i^2) / (2a)
```

Implemented as:

```cpp
d = (v_f * v_f - v_i * v_i) /
    (2.0 * a);
```

## Purpose

Computes distance traveled during constant acceleration or deceleration.

Kinematic equation:

:contentReference[oaicite:0]{index=0}

---

# Graphical Sketch – Decel to Stop Profile

```text
Velocity

^
|\
| \
|  \
|   \________
|            \
|             \
|              \____
|
+--------------------------------> Distance
```

Stages:

1. Smooth deceleration
2. Constant slow-speed region
3. Final braking to stop

---

# Reverse Speed Construction

If stopping distance exceeds path length:

```cpp
if (brake_distance + decel_distance > path_length)
```

the planner generates the velocity profile backwards from zero speed.

---

# Backward Speed Computation

## Implementation

```cpp
auto vi =
    calc_final_speed(vf, -_a_max, dist);
```

## Purpose

Ensures the vehicle reaches exactly zero speed at the stop line.

---

# 2. Nominal Velocity Profile

## Function

```cpp
nominal_trajectory(...)
```

## Purpose

Generates smooth acceleration or deceleration toward a desired cruising speed.

---

# Nominal Profile Structure

```text
Current Speed
      ↓
Smooth Acceleration/Deceleration
      ↓
Desired Cruise Speed
      ↓
Constant Velocity
```

---

# Acceleration Distance

## Implementation

```cpp
accel_distance =
    calc_distance(
        start_speed,
        desired_speed,
        _a_max);
```

## Purpose

Computes the distance required to reach target speed.

---

# Final Speed Equation

## Implementation

```cpp
double disc =
    v_i * v_i + 2.0 * a * d;
```

```cpp
v_f = std::sqrt(disc);
```

## Purpose

Computes velocity after traveling distance `d` under constant acceleration.

Kinematic equation:

:contentReference[oaicite:1]{index=1}

---

# Negative Discriminant Handling

## Implementation

```cpp
if (disc <= 0.0)
    v_f = 0.0;
```

## Purpose

Prevents invalid square root operations.

A negative discriminant means the vehicle cannot physically achieve the requested motion.

---

# Infinity and NaN Handling

## Implementation

```cpp
else if (
    disc == infinity ||
    std::isnan(disc))
```

## Purpose

Prevents invalid numerical behavior during planning.

---

# Graphical Sketch – Nominal Profile

```text
Velocity

^
|           ____________
|          /
|         /
|        /
|_______/
|
+--------------------------------> Distance
```

Stages:

1. Smooth acceleration/deceleration
2. Constant desired speed cruising

---

# Ramp End Detection

## Implementation

```cpp
while (
    ramp_end_index < (spiral.size() - 1) &&
    (distance < accel_distance))
```

## Purpose

Determines where acceleration phase ends.

---

# Time Assignment

## Implementation

```cpp
time_step =
    std::fabs(vf - vi) / _a_max;
```

## Purpose

Assigns timestamps to trajectory points.

This produces a time-parameterized trajectory.

---

# Interpolation Step

## Implementation

```cpp
trajectory[0] = interpolated_state;
```

## Purpose

Prevents the controller from getting stuck at the initial state.

Adds smoother trajectory startup behavior.

---

# Key Concepts

- Velocity planning
- Trapezoidal velocity profiles
- Kinematic equations
- Constant acceleration motion
- Time-parameterized trajectories
- Smooth stopping behavior

---

# Summary

The velocity planner now:

- generates smooth stopping profiles
- computes nominal cruising trajectories
- applies kinematic motion equations
- assigns velocity at each trajectory point
- produces time-parameterized motion plans

This module enables smooth and physically feasible longitudinal motion for autonomous driving.
