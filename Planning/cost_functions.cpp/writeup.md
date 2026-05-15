
# Cost Functions

## Overview

This file implements the main trajectory evaluation cost functions used in the lattice planner.

The planner evaluates candidate trajectories based on:

- goal accuracy
- collision safety
- distance from lane center
- trajectory feasibility

The trajectory with the lowest total cost is selected.

---

# 1. Goal Difference Cost

## Function

```cpp
double diff_cost(...)
```

## Purpose

Penalizes trajectories that deviate from the desired goal state.

The cost evaluates:

- position
- velocity
- acceleration

differences between the generated trajectory and the desired goal.

---

## Implementation

```cpp
vector<double> evals =
    evaluate_f_and_N_derivatives(coeff, duration, 2);
```

Trajectory values are evaluated at the final time.

---

## Cost Computation

```cpp
double diff = fabs(evals[i] - goals[i]);
cost += logistic(diff / sigma[i]);
```

Each error term is normalized using sigma values.

The logistic function converts the error into a smooth bounded cost.

---

# 2. Collision Cost

## Function

```cpp
double collision_circles_cost_spiral(...)
```

## Purpose

Checks whether a generated spiral trajectory collides with obstacles.

This is one of the most important safety checks in the planner.

---

# Circle-Based Collision Model

The vehicle is approximated using multiple collision circles.

```cpp
constexpr std::array<float, 3> CIRCLE_OFFSETS
```

defines circle positions along the vehicle body.

---

# Ego Vehicle Circle Placement

## Implementation

```cpp
auto circle_center_x =
    cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);

auto circle_center_y =
    cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);
```

## Purpose

Places collision circles along the vehicle heading direction.

This approximates the vehicle footprint during motion.

---

# Obstacle Circle Placement

## Implementation

```cpp
auto actor_center_x =
    obst.location.x +
    CIRCLE_OFFSETS[c2] * std::cos(actor_yaw);

auto actor_center_y =
    obst.location.y +
    CIRCLE_OFFSETS[c2] * std::sin(actor_yaw);
```

## Purpose

Approximates obstacle geometry using collision circles.

---

# Circle Distance Computation

## Implementation

```cpp
double dist = std::sqrt(
    std::pow(circle_center_x - actor_center_x, 2) +
    std::pow(circle_center_y - actor_center_y, 2));
```

## Purpose

Computes Euclidean distance between ego and obstacle circles.

Distance equation:


::contentReference[oaicite:0]{index=0}


---

# Collision Detection

## Implementation

```cpp
collision =
    (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
```

## Purpose

Detects overlap between collision circles.

If circles overlap:

- collision exists
- trajectory becomes invalid

---

# Collision Cost Output

## Implementation

```cpp
return (collision) ? COLLISION : 0.0;
```

## Purpose

Returns a very large cost for colliding trajectories.

This ensures unsafe paths are rejected.

---

# 3. Distance to Main Goal Cost

## Function

```cpp
double close_to_main_goal_cost_spiral(...)
```

## Purpose

Encourages trajectories that remain close to the lane centerline.

Collision-free paths closer to the main goal are preferred.

---

# Final Point Distance Computation

## Implementation

```cpp
auto delta_x =
    main_goal.location.x - spiral[n - 1].x;

auto delta_y =
    main_goal.location.y - spiral[n - 1].y;

auto delta_z =
    main_goal.location.z - spiral[n - 1].z;
```

---

# Euclidean Distance

## Implementation

```cpp
auto dist = std::sqrt(
    (delta_x * delta_x) +
    (delta_y * delta_y) +
    (delta_z * delta_z));
```

## Purpose

Measures how far the spiral endpoint is from the center-line goal.

Smaller distances produce lower cost.

---

# Logistic Cost

## Implementation

```cpp
auto cost = logistic(dist);
```

## Purpose

Smoothly penalizes trajectories farther from the lane center.

This avoids harsh discontinuities in trajectory ranking.

---

# Why Cost Functions Matter

The lattice planner generates multiple candidate trajectories.

Cost functions allow the planner to:

- rank trajectories
- reject unsafe paths
- prefer smoother solutions
- remain near lane center
- satisfy goal constraints

Instead of choosing:

```text
Any valid path
```

the planner chooses:

```text
The safest and most optimal path
```

---

# Key Concepts

- Trajectory optimization
- Collision checking
- Circle-based vehicle approximation
- Euclidean distance metrics
- Logistic cost scaling
- Lattice trajectory evaluation

---

# Summary

The cost functions evaluate candidate trajectories using:

- goal accuracy
- collision safety
- lane-center preference

These costs allow the autonomous planner to safely select the best trajectory among multiple generated spiral paths.
