
# Autonomous Driving Planning and Control Project

## Overview

This project implements a simplified autonomous driving stack in the CARLA simulator using:

- Behavior Planning
- Motion Planning
- Velocity Planning
- Collision Checking

The system generates safe and smooth trajectories for autonomous driving.

---

# Features

## Behavior Planner FSM

Handles:

- lane following
- stop-line handling
- junction detection
- state transitions

States:

```text
FOLLOW_LANE
DECEL_TO_STOP
STOPPED
```

---

## Motion Planner

- Generates offset goals
- Creates cubic spiral trajectories
- Performs collision checking
- Selects best trajectory using cost functions

---

## Velocity Planner

Generates:

- cruising velocity profiles
- deceleration-to-stop profiles
- smooth longitudinal motion

Uses motion equations:

:contentReference[oaicite:0]{index=0}

---

## PID Controller

Controls:

- steering
- throttle
- brake

to follow the generated trajectory.

---

# Technologies Used

- C++
- CARLA Simulator
- CMake
- Eigen
- STL

---

# Project Structure

```text
.
├── behavior_planner_FSM.cpp
├── motion_planner.cpp
├── velocity_profile_generator.cpp
├── cost_functions.cpp
├── planning_params.h
├── pid_controller/
├── run_main_pid.sh
├── install-ubuntu.sh
└── CMakeLists.txt
```

---

# Setup Instructions

## Step 1 — Clone Repository

```bash
git clone https://github.com/udacity/nd013-c6-control-starter.git
```

```bash
cd nd013-c6-control-starter/project
```

---

## Step 2 — Start CARLA Simulator

Open a terminal and run:

```bash
/opt/carla-simulator/CarlaUE4.sh
```

---

## Step 3 — Install Dependencies

Open another terminal:

```bash
cd nd013-c6-control-starter/project
```

Run:

```bash
./install-ubuntu.sh
```

This installs:

- libuv1-dev
- libssl-dev
- libz-dev
- uWebSockets

---

## Step 4 — Update Project Files

Go to:

```bash
cd pid_controller/
```

Update TODOs in:

```text
pid_controller.h
pid_controller.cpp
main.cpp
```

---

## Step 5 — Build the Project

From the `pid_controller/` directory:

```bash
cmake .
```

Compile:

```bash
make
```

Re-run `make` after every code change.

---

## Step 6 — Run the Project

Return to project directory:

```bash
cd ..
```

Run:

```bash
./run_main_pid.sh
```

---

# Common Errors

## Silent Execution Failure

Stop using:

```bash
CTRL + C
```

and try again.

---

## Address Already in Use

Find running CARLA processes:

```bash
ps -aux | grep carla
```

Kill process:

```bash
kill <process_id>
```

---

# Planning Pipeline

```text
Behavior Planner
      ↓
Motion Planner
      ↓
Spiral Generation
      ↓
Collision Checking
      ↓
Velocity Planning
      ↓
PID Control
```

---

# Summary

This project demonstrates a complete autonomous driving planning pipeline including:

- behavior planning
- trajectory generation
- collision checking
- velocity planning
  

using lattice-based planning in the CARLA simulator.
