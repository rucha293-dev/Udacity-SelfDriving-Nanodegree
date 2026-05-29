# Control and Trajectory Tracking for Autonomous Vehicle

In this project, you will apply the skills you have acquired in this course to design a Proportional-Integral-Derivative (PID) controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.
# Instructions
The sections ahead will guide you through the steps to build and run the project. 

## Step 1. Log into VM Workspace

Open the VM workspace and log into the VM to practice the current project. 
Once you log into the VM, open a Terminal window. 

<br/><br/>

## Step 2. Clone the Repository

Fork the repository to your Github account and clone the repository to the workspace using the commands below. 

```bash
git clone https://github.com/udacity/nd013-c6-control-starter.git
```

Change to the project directory.
```bash
cd nd013-c6-control-starter/project
```

<br/><br/>

## Step 3. Review the starter files
You will find the following files in the project directory.

```bash
.
├── cserver_dir
├── install-ubuntu.sh
├── manual_control.py
├── pid_controller/     # TODO Files
├── plot_pid.py
├── run_main_pid.sh
├── simulatorAPI.py
├── steer_pid_data.txt
└── throttle_pid_data.txt
```

<br/><br/>

## Step 4. Start the Carla Server
Start the Carla server by executing the following shell script. 
```bash
/opt/carla-simulator/CarlaUE4.sh
```


<br/><br/>

## Step 5. Install Dependencies
Open another Terminal tab, and change to the **nd013-c6-control-starter/project**  directory. Execute the following shell script to install the project-specific dependencies. 
```bash
./install-ubuntu.sh
```
This file will install utilities such as, `libuv1-dev`, `libssl-dev`, `libz-dev`, `uWebSockets`. 

<br/><br/>

## Step 6. Update the Project Code

Change to the **pid_controller/** directory.
```bash
cd pid_controller/
```
Before you start coding, we strongly recommend you look at the rubric in your classroom, against which the human Mentor will review your submission. Your submission must satisfy all rubric criteria to pass the project; otherwise, the Mentor may ask you to re-submit. 


Update the following files as per the classroom instructions. You will TODO markers as well in these files. 

- **pid_controller.h**
- **pid_controller.cpp**
- **main.cpp**


<br/>

> **Important**: At this moment, it is important to save your work and push it back to the remote Github repository. 

<br/><br/>

### Update Notes
In the previous version of the project starter code, we had **libcarla-install/** and **rpclib/** directories inside the **pid_controller/** directory. But, those directories are no longer needed in the current version of the starter code because the current **CMakeLists.txt** file has corresponding `includes` and `libs` added at `/opt/carla-source`.

To give some old context, when we had **rpclib/** directory inside the starter files, we used to compile the **rpclib** library using the following commands. 
```bash
cd pid_controller/
rm -rf rpclib
git clone https://github.com/rpclib/rpclib.git
```
This library is a **msgpack-rpc** library written using modern C++. The goal of building this library was to provide a simple RPC solution. However, all of the above-mentioned steps are **no longer needed** in the current version of the project strarter code. 

<br/><br/>

## Step 7. Build and Execute the Project

When you finish updating the project files, you can execute the project using the commands below. 

```bash
# Build the project
# Run the following commands from the pid_controller/ directory
cmake .
# The command below compiles your c++ code. Run it after each time you edit the CPP or Header files
make
```

```bash
# Run the project
cd ..
# Run the following commands from the nd013-c6-control-starter/project directory
./run_main_pid.sh
```
If the execution fails silently, you can use **ctrl + C** to stop, and try again. 

Another possible error you may get is `bind failed. Error: Address already in use`. In such a case, you can kill the process occupying the required port using the commands below.

```bash
ps -aux | grep carla
# Use the IDs displayed in the output of the last command. 
kill id     
```

<br/><br/>
# PID Controller Evaluation and Analysis

## 1. Plot Results

Two plots were generated from the simulation run:

**Steering PID Plot** — shows Error Steering (blue) and Steering Output (orange) over ~600 iterations.

**Throttle PID Plot** — shows Error Throttle (blue), Brake Output (orange), and Throttle Output (green) over ~600 iterations.

---

## 2. Analyzing the Plots

### Steering Plot

The steering error (blue) starts near zero on the straight road, then spikes to around **-1.1 to -1.5 radians** at iterations ~130 and ~450 — these correspond to the two obstacle vehicles encountered on the path. The steering output (orange) responds by saturating at approximately **-0.5**, showing the PID is commanding maximum left avoidance steer. After each obstacle, the error gradually returns toward zero as the car re-centers on the lane. The early iterations (0–100) show small oscillations in the output caused by the derivative term responding to rapid changes in the heading error as the car navigates the initial trajectory.

### Throttle Plot

The throttle error (blue) stays consistently positive at approximately **1.0 m/s**, meaning the car is always slightly below the desired speed — expected behavior since the planner targets the speed limit and the car takes time to accelerate. The throttle output (green) is smooth at **~0.2–0.3**, indicating a well-tuned throttle PID with no aggressive braking or acceleration spikes. The brake output (orange) remains flat at zero throughout, confirming the car never needed to brake hard — obstacle avoidance was handled entirely through steering.

---

## 3. Role of Each PID Component

### Proportional (Kp)

Kp produces a control output directly proportional to the current error. For steering, a higher Kp causes a sharper turn when the heading error is large (e.g. when an obstacle forces the trajectory to curve). For throttle, Kp determines how aggressively the car accelerates toward the desired speed. If Kp is too low, the car responds too slowly and collides; too high and it overshoots and oscillates.

### Integral (Ki)

Ki accumulates error over time and corrects persistent steady-state offsets. In this implementation Ki was kept small (0.001) for both controllers. For steering, it corrects any long-term lateral drift from the lane center that Kp alone cannot eliminate. For throttle, it ensures the car eventually reaches the exact desired speed even if Kp leaves a small residual error.

### Derivative (Kd)

Kd acts on the rate of change of error and provides damping. For steering, Kd is the most critical term for returning the car to center after obstacle avoidance — a high Kd (2.5 in our final tuning) means as soon as the heading error starts decreasing (car turning back toward center), a strong counter-steer is applied to snap it back quickly and prevent overshooting into the opposite lane or wall. Without sufficient Kd the car would oscillate left-right indefinitely after each avoidance maneuver.

---

## 4. Critical Analysis

### How would you design a way to automatically tune the PID parameters?

The **Ziegler–Nichols method** was used as the basis for tuning in this project. To automate this:

1. Run the simulation with Ki = 0 and Kd = 0, then incrementally increase Kp until the output oscillates with constant amplitude. This gives the **ultimate gain (Ku)**.
2. Measure the oscillation period **Tu**.
3. Compute gains automatically:

   - Kp = 0.6 × Ku
   - Ki = 1.2 × Ku / Tu
   - Kd = 0.075 × Ku × Tu

A more modern approach would use **Twiddle (coordinate ascent)**, which iteratively perturbs each gain, keeps changes that reduce total error, and converges toward an optimum. This can be fully automated by running multiple simulation episodes and minimizing a cost function such as total cross-track error or number of collisions.

Another alternative is **reinforcement learning (RL)**, such as PPO or DDPG, which can learn optimal gains—or even replace the PID controller entirely—through repeated simulation trials.

### PID Controller: Model-Free — Pros and Cons

A PID controller is **model-free**, meaning it does not require explicit knowledge of the vehicle dynamics (mass, inertia, tire forces, etc.). It only reacts to measured error signals.

#### Advantages

- Simple to implement and tune
- Computationally efficient
- Robust to modeling inaccuracies
- Easily transferable across different vehicles with minor retuning

#### Disadvantages

- Cannot predict future errors
- Performance degrades at high speeds and sharp curves
- Requires manual tuning
- Susceptible to integral windup

### Comparison with Model Predictive Control (MPC)

#### Advantages of MPC

- Predicts future vehicle behavior over a planning horizon
- Handles constraints naturally (steering limits, acceleration bounds)
- Optimizes steering and throttle simultaneously
- Better performance in complex scenarios

#### Disadvantages of MPC

- Requires an accurate vehicle model
- Higher computational cost
- Sensitive to modeling errors

### How Would You Improve the PID Controller?

1. **Replace PID with MPC**
   - MPC can optimize steering and speed jointly while considering future trajectory constraints.

2. **Add Feedforward Control**
   - Use planned trajectory curvature to generate steering commands before large errors occur.

3. **Adaptive Gain Scheduling**
   - Modify PID gains dynamically based on speed, curvature, or obstacle proximity.

4. **Integral Anti-Windup**
   - Clamp the integral term whenever the controller output saturates to prevent excessive overshoot once saturation ends.

---

## Conclusion

The PID controller successfully guided the vehicle through the motion planning scenario while maintaining stable speed control and obstacle avoidance. The steering controller handled obstacle-induced trajectory deviations effectively, while the throttle controller maintained smooth acceleration without requiring braking. Although PID provides a simple and computationally efficient solution, future improvements such as feedforward control, adaptive gains, anti-windup mechanisms, or Model Predictive Control could further enhance performance and robustness.
