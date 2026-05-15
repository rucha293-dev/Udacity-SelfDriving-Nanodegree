
# Motion Planner Implementation

## Overview

This file implements the lattice-based motion planning pipeline for autonomous driving.

The planner is responsible for:

- transforming goals into ego coordinates
- generating offset goals
- generating cubic spiral trajectories
- validating candidate spirals
- transforming spirals back to global frame
- evaluating trajectory costs
- selecting the best path

The planner generates multiple candidate trajectories and selects the safest collision-free spiral.

---

# Motion Planning Pipeline

```text
Global Goal
      ↓
Transform to Ego Frame
      ↓
Generate Offset Goals
      ↓
Generate Candidate Spirals
      ↓
Discretize/Sample Spirals
      ↓
Collision Checking
      ↓
Cost Evaluation
      ↓
Select Best Spiral
      ↓
Transform Back to Global Frame
```

---

# 1. Goal Transformation to Ego Frame

## Function

```cpp
get_goal_state_in_ego_frame(...)
```

## Purpose

Transforms the goal from global coordinates into the ego vehicle frame.

This simplifies planning because the vehicle becomes:

```text
Position = (0,0)
Yaw = 0
```

---

# Translation

## Implementation

```cpp
goal_state_ego_frame.location.x -= ego_state.location.x;
goal_state_ego_frame.location.y -= ego_state.location.y;
```

## Purpose

Moves the ego vehicle to the origin.

---

# Rotation

## Implementation

```cpp
auto theta_rad = -ego_state.rotation.yaw;
```

```cpp
goal_state_ego_frame.location.x =
    cos_theta * goal_x - sin_theta * goal_y;

goal_state_ego_frame.location.y =
    sin_theta * goal_x + cos_theta * goal_y;
```

## Purpose

Rotates coordinates so the ego heading becomes zero.

Rotation equation:

:contentReference[oaicite:0]{index=0}

---

# Yaw Normalization

## Implementation

```cpp
goal_state_ego_frame.rotation.yaw =
    utils::keep_angle_range_rad(
        goal_state_ego_frame.rotation.yaw,
        -M_PI,
        M_PI);
```

## Purpose

Keeps heading within:

```text
[-π, π]
```

to improve optimizer stability.

---

# 2. Offset Goal Generation

## Function

```cpp
generate_offset_goals(...)
```

## Purpose

Generates multiple laterally shifted goals around the center-line goal.

This improves:

- obstacle avoidance
- trajectory diversity
- planner robustness

---

# Perpendicular Direction

## Implementation

```cpp
auto yaw =
    goal_state.rotation.yaw + M_PI / 2.0;
```

## Purpose

Computes a direction perpendicular to the lane heading.

Adding:

\[
\frac{\pi}{2}
\]

rotates the heading by 90°.

---

# Offset Computation

## Implementation

```cpp
float offset =
    (i - (int)(_num_paths / 2)) * _goal_offset;
```

## Purpose

Generates symmetric lateral offsets.

Example for 5 paths:

```text
[-2] [-1] [0] [+1] [+2]
```

---

# Offset Goal Placement

## Implementation

```cpp
goal_offset.location.x +=
    offset * std::cos(yaw);

goal_offset.location.y +=
    offset * std::sin(yaw);
```

## Purpose

Places goals laterally relative to the road direction.

---

# Goal Validation

## Function

```cpp
valid_goal(...)
```

## Purpose

Ensures generated offset goals remain within valid lateral bounds.

---

# 3. Spiral Generation

## Function

```cpp
generate_spirals(...)
```

## Purpose

Generates cubic spiral trajectories connecting:

```text
Current State → Offset Goal
```

Each spiral becomes a candidate path.

---

# Spiral Start State

## Implementation

```cpp
start.x = 0.0;
start.y = 0.0;
start.theta = 0.0;
```

## Purpose

Since planning occurs in ego frame, the vehicle starts at the origin.

---

# Spiral End State

## Implementation

```cpp
end.x = goal.location.x;
end.y = goal.location.y;
end.theta = goal.rotation.yaw;
```

## Purpose

Defines the target offset goal for trajectory generation.

---

# Arc Length

## Implementation

```cpp
end.s = std::sqrt(
    (end.x * end.x) +
    (end.y * end.y) +
    (end.z * end.z));
```

## Purpose

Computes approximate spiral length.

Distance equation:

:contentReference[oaicite:1]{index=1}

---

# Discretized Spiral Sampling

## Implementation

```cpp
_cubic_spiral.GetSampledSpiral(
    P_NUM_POINTS_IN_SPIRAL,
    spiral);
```

## Purpose

Returns sampled trajectory points along the spiral.

Example:

```text
(x0, y0)
(x1, y1)
(x2, y2)
...
(xN, yN)
```

Sampling is necessary for:

- collision checking
- trajectory evaluation
- visualization

---

# Spiral Validation

## Function

```cpp
valid_spiral(...)
```

## Purpose

Ensures the generated spiral actually reaches the intended offset goal.

---

# Endpoint Error Computation

## Implementation

```cpp
auto dist = std::sqrt(
    (delta_x * delta_x) +
    (delta_y * delta_y));
```

## Purpose

Measures distance between:

- spiral endpoint
- target goal

The spiral is valid if:

```cpp
dist < 0.1
```

---

# 4. Transform Spirals Back to Global Frame

## Function

```cpp
transform_spirals_to_global_frame(...)
```

## Purpose

Converts ego-frame spirals back into world coordinates.

Required for:

- simulation
- visualization
- controller execution

---

# Global Coordinate Transformation

## Implementation

```cpp
new_path_point.x =
    ego_state.location.x +
    path_point.x * cos(ego_state.rotation.yaw) -
    path_point.y * sin(ego_state.rotation.yaw);
```

```cpp
new_path_point.y =
    ego_state.location.y +
    path_point.x * sin(ego_state.rotation.yaw) +
    path_point.y * cos(ego_state.rotation.yaw);
```

## Purpose

Applies inverse rotation and translation.

---

# 5. Cost Evaluation

## Function

```cpp
calculate_cost(...)
```

## Purpose

Computes total trajectory cost.

The planner selects the spiral with the minimum cost.

---

# Collision Cost

## Implementation

```cpp
cf::collision_circles_cost_spiral(...)
```

## Purpose

Rejects unsafe trajectories that collide with obstacles.

---

# Distance-to-Goal Cost

## Implementation

```cpp
cf::close_to_main_goal_cost_spiral(...)
```

## Purpose

Prefers trajectories close to the lane centerline.

---

# 6. Best Spiral Selection

## Function

```cpp
get_best_spiral_idx(...)
```

## Purpose

Evaluates all candidate spirals and selects:

```text
Lowest Cost Collision-Free Path
```

Colliding spirals are discarded.

---

# Key Concepts

- Lattice planning
- Coordinate transformations
- Offset goal generation
- Cubic spiral trajectories
- Spiral discretization
- Collision checking
- Cost-based path selection

---

# Summary

The motion planner now:

- transforms goals into ego coordinates
- generates lateral offset goals
- creates cubic spiral trajectories
- discretizes spirals for collision checking
- evaluates trajectory costs
- selects the safest valid path

This forms the core local trajectory planning pipeline used in autonomous driving systems.
