# Instructions to Execute the Project

1. Navigate to the **c3-project** directory containing the project starter files.
    ```bash
    cd nd0013_cd2693_Exercise_Starter_Code/Lesson_7_Project_Scan_Matching_Localization/c3-project
    ```


2. Review the starter files. You must find the following files in your current working directory.
    ```bash
    .
    ├── CMakeLists.txt
    ├── README.md
    ├── c3-main.cpp
    ├── helper.cpp
    ├── helper.h
    ├── make-libcarla-install.sh
    ├── map.pcd
    ├── map_loop.pcd
    ├── rpclib
    └── run_carla.sh
    ```


3. Ensure that the **libcarla-install/** folder is present in your current working directory. This folder contains the static binaries built for the target VM workspace environment. You will need to regenerate it using the following commands in order:
    ```bash
    chmod +x make-libcarla-install.sh
    ./make-libcarla-install.sh
    ```



4. Update the ** c3-main.cpp** file per the `TODO` markers and the classroom instructions. 


5. Compile the project using the following commands. 

    ```bash
    cmake .
    make
    ```
    These steps will generate the **clooud_loc** executable. 


6. Open a new Terminal tab and execute the following command to start the simulator.

    ```bash
    ./run_carla.sh
    ```  


7. Open another Terminal tab and execute the following to run the project.
    ```bash
    ./cloud_loc 
    ```
If you encounter core dump on start up, just rerun and try again. Crash doesn't happen more than a couple of times. 


# Point Cloud Processing

## Voxel Grid Filtering

The raw LiDAR scan is filtered to reduce noise and computational load.

### Purpose
- Downsample point clouds
- Improve NDT performance
- Reduce runtime cost

```cpp
pcl::VoxelGrid<PointT> vg;
vg.setLeafSize(0.7f,0.7f,0.7f);
```

---

# NDT Localization

The filtered LiDAR scan is aligned against a prebuilt map using:

\[
T^* = \arg\max_T p(\text{Scan} \mid \text{Map})
\]

The algorithm estimates the transformation that best aligns the current scan with the reference map.

## NDT Parameters

```cpp
ndt.setResolution(1.0);
ndt.setStepSize(0.3);
ndt.setTransformationEpsilon(0.0001);
ndt.setMaximumIterations(50);
```

---

# Pose Estimation

The estimated transformation matrix is used to compute:

- Vehicle X position
- Vehicle Y position
- Vehicle yaw angle

```cpp
pose.position.x = transformation(0,3);
pose.position.y = transformation(1,3);

pose.rotation.yaw =
    atan2(transformation(1,0),
           transformation(0,0));
```

---

# Fitness Score Rejection

Bad alignments are rejected using the NDT fitness score.

```cpp
if(ndt.hasConverged() &&
   ndt.getFitnessScore() < 1.5)
```

This prevents unstable localization updates.

---

# Drift Reset Logic

If localization error becomes too large, the pose estimate is reset.

```cpp
if(poseError > 5.0)
```

This helps recover from divergence.
