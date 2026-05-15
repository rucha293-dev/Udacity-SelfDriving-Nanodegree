
# Behavior Planner FSM

## Overview

This project implements a basic Finite State Machine (FSM) behavior planner for autonomous driving.

The planner supports:

- Lane following
- Stop-line detection
- Deceleration before intersections
- Full stop handling
- Resuming lane following

---

# FSM Flow

```text
FOLLOW_LANE
     ↓
DECEL_TO_STOP
     ↓
STOPPED
     ↓
FOLLOW_LANE
```

---

# Implemented Features

## 1. Dynamic Lookahead Distance

### Implementation

```cpp
auto look_ahead_distance =
    (velocity_mag * velocity_mag) / (2.0 * P_MAX_ACCEL);
```

### Purpose

Computes a speed-dependent lookahead distance for smoother planning.

---

## 2. Goal Generation on Lane Center

### Implementation

```cpp
get_closest_waypoint_goal(...)
```

### Purpose

Generates a goal waypoint along the lane centerline.

---

## 3. Transition to DECEL_TO_STOP

### Implementation

```cpp
if (is_goal_in_junction)
    _active_maneuver = DECEL_TO_STOP;
```

### Purpose

Triggers controlled braking before intersections.

---

## 4. Stop-Line Buffer

### Implementation

```cpp
auto ang = goal.rotation.yaw + M_PI;

goal.location.x += _stop_line_buffer * cos(ang);
goal.location.y += _stop_line_buffer * sin(ang);
```

### Purpose

Moves the stop goal slightly behind the stop line.

---

## 5. Stop Goal Velocity

### Implementation

```cpp
goal.velocity.x = 0.0;
goal.velocity.y = 0.0;
goal.velocity.z = 0.0;
```

### Purpose

Ensures the vehicle comes to a complete stop.

---

## 6. Lane Following Velocity

### Implementation

```cpp
goal.velocity.x =
    _speed_limit * cos(goal.rotation.yaw);

goal.velocity.y =
    _speed_limit * sin(goal.rotation.yaw);
```

### Purpose

Aligns vehicle velocity with lane direction.

---

## 7. Maintain Goal During DECEL_TO_STOP

### Implementation

```cpp
goal = _goal;
```

### Purpose

Keeps a fixed stopping target during braking.

---

## 8. Distance-Based Stop Detection

### Implementation

```cpp
auto distance_to_stop_sign =
    utils::magnitude(goal.location - ego_state.location);
```

```cpp
if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE)
```

### Purpose

Uses distance instead of speed to detect arrival at the stop line.

---

## 9. Transition to STOPPED

### Implementation

```cpp
_active_maneuver = STOPPED;
```

### Purpose

Keeps the vehicle stationary at the stop line.

---

## 10. Maintain Goal During STOPPED

### Implementation

```cpp
goal = _goal;
```

### Purpose

Prevents goal updates while waiting.

---

## 11. Transition Back to FOLLOW_LANE

### Implementation

```cpp
_active_maneuver = FOLLOW_LANE;
```

### Purpose

Resumes normal lane-following behavior.

---

# Key Concepts

- Finite State Machines (FSM)
- Autonomous behavior planning
- Dynamic lookahead generation
- Stop-line handling
- Goal persistence
- State transitions

---

# Summary

The completed FSM planner enables the autonomous vehicle to:

- Follow lanes
- Stop safely at intersections
- Wait at stop lines
- Resume driving autonomously

using a structured behavior-planning pipeline.
