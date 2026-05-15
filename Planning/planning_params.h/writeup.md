
# Planning Parameters

## Overview

This file defines the main planning and behavior constants used by the autonomous driving stack.

These parameters control:

- goal generation
- lookahead behavior
- stopping logic
- lattice planning
- trajectory generation
- comfort constraints

---

# Path Generation Parameters

## Number of Paths

```cpp
#define P_NUM_PATHS 5
```

### Purpose

Defines the number of offset goals generated around the center-line goal.

The value should always be odd so the planner has:

```text
Left Goals + Center Goal + Right Goals
```

Example:

```text
[-2] [-1] [0] [+1] [+2]
```

Where:

- `0` is the center goal
- negative indices are left offsets
- positive indices are right offsets

### Why 5?

Using 5 paths provides:

- enough trajectory diversity
- reasonable computation cost
- symmetric goal generation

Too many paths increase computation.

Too few paths reduce obstacle avoidance capability.

---

## Goal Offset Distance

```cpp
#define P_GOAL_OFFSET 1.0
```

### Purpose

Defines lateral spacing between neighboring offset goals.

Larger offsets:

- create wider trajectories
- may skip feasible paths

Smaller offsets:

- increase path similarity
- increase computational load

---

# Lookahead Parameters

## Minimum Lookahead

```cpp
#define P_LOOKAHEAD_MIN 8.0
```

### Purpose

Prevents unstable planning at low speeds.

---

## Maximum Lookahead

```cpp
#define P_LOOKAHEAD_MAX 20.0
```

### Purpose

Limits excessive planning distance at high speeds.

---

## Lookahead Time

```cpp
#define P_LOOKAHEAD_TIME 1.5
```

### Purpose

Fallback lookahead calculation based on speed and time.

---

# Vehicle Dynamics Parameters

## Maximum Acceleration

```cpp
#define P_MAX_ACCEL 1.5
```

### Purpose

Used for stopping-distance calculations and smooth planning.

---

## Speed Limit

```cpp
#define P_SPEED_LIMIT 3.0
```

### Purpose

Desired nominal driving speed.

---

## Slow Speed

```cpp
#define P_SLOW_SPEED 1.0
```

### Purpose

Defines reduced-speed behavior when needed.

---

# Stopping Parameters

## Stop Line Buffer

```cpp
#define P_STOP_LINE_BUFFER 0.5
```

### Purpose

Stops the vehicle slightly before the stop line.

---

## Stop Threshold Speed

```cpp
#define P_STOP_THRESHOLD_SPEED 0.02
```

### Purpose

Defines near-zero velocity threshold.

---

## Required Stop Time

```cpp
#define P_REQ_STOPPED_TIME 1.0
```

### Purpose

Minimum waiting time before resuming motion.

---

## Stop Threshold Distance

```cpp
#define P_STOP_THRESHOLD_DISTANCE \
  P_LOOKAHEAD_MIN / P_NUM_POINTS_IN_SPIRAL * 2
```

### Purpose

Distance threshold used to detect arrival at stop line.

---

# Spiral Trajectory Parameters

## Number of Spiral Points

```cpp
#define P_NUM_POINTS_IN_SPIRAL 20
```

### Purpose

Defines trajectory resolution.

Higher values:

- smoother spirals
- better accuracy
- higher computation cost

Lower values:

- faster computation
- rougher trajectories

---

# Collision Checking Parameters

## Collision Circle Offsets

```cpp
constexpr std::array<float, 3> CIRCLE_OFFSETS =
{-1.0, 1.0, 3.0};
```

### Purpose

Defines circle locations along the vehicle body.

---

## Collision Circle Radii

```cpp
constexpr std::array<float, 3> CIRCLE_RADII =
{1.5, 1.5, 1.5};
```

### Purpose

Defines collision boundaries for obstacle checking.

---

# Trajectory Perturbation Parameters

## Sigma Values

```cpp
constexpr std::array<float, 3> SIGMA_X
constexpr std::array<float, 3> SIGMA_Y
constexpr std::array<float, 3> SIGMA_YAW
```

### Purpose

Used for generating perturbed candidate trajectories.

These help the planner explore multiple feasible motions.

---

# Time Parameters

## Time Step

```cpp
constexpr double dt = 0.05;
```

### Purpose

Simulation and trajectory discretization timestep.

---

## Maneuver Time Limits

```cpp
constexpr double MIN_MANEUVER_TIME
constexpr double MAX_MANEUVER_TIME
```

### Purpose

Limits trajectory duration.

---

# Comfort Constraints

## Maximum Jerk

```cpp
CONFORT_MAX_LAT_JERK
CONFORT_MAX_LON_JERK
```

### Purpose

Limits sudden acceleration changes for passenger comfort.

---

## Maximum Acceleration

```cpp
CONFORT_MAX_LON_ACCEL
CONFORT_MAX_LAT_ACCEL
```

### Purpose

Keeps generated trajectories dynamically feasible and comfortable.

---

# Summary

The planning parameters define the behavior and constraints of the autonomous driving planner.

These constants influence:

- path generation
- obstacle avoidance
- stopping behavior
- trajectory smoothness
- computational cost
- passenger comfort

Proper tuning of these values is critical for stable and safe autonomous driving.
