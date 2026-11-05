# Quadruped Robot with Manipulator — ROS 2 Workspace

Software workspace for my master's final project (TFM), documented in [UPCommons, Universitat Politècnica de Catalunya](https://upcommons.upc.edu/entities/publication/f53c69dd-fdc5-4073-99e1-0c4ce5c5da53).

The project combines a quadruped platform, a robotic arm, a Python control interface, inertial sensing and serial communication with embedded hardware.

## Components

| Package | Contents |
| --- | --- |
| `quadruped_arm_motion` | Robot models, meshes, RViz configuration, launch files, motion and trajectory scripts, simulation experiments and tag pose extraction |
| `Quad_control_gui` | Python control GUI and inverse-kinematics code |
| `mpu_pub` | IMU acquisition, calibration and estimation experiments, plus serial motor communication |
| `robot_interfaces` | Custom ROS messages for commands, joint angles, motion and sensor data |

External integrations include AprilTag, image pipeline, USB camera and teleoperation code. Their presence does not establish that every integration works in the current checkout.

## Architecture

```mermaid
flowchart LR
    GUI[Control GUI] --> ROS[ROS 2 commands and custom messages]
    ROS --> SERIAL[Serial motor interface]
    SERIAL --> HW[Embedded controller and actuators]
    IMU[Inertial sensor] --> EST[Acquisition and estimation nodes]
    EST --> ROS
    MODEL[URDF / Xacro models] --> VIEW[Robot visualization]
```

This diagram describes the main software areas in the source; the exact runtime node graph depends on the chosen launch file.

## Getting started

Use a Linux environment with ROS 2, colcon and rosdep installed. The exact validated OS, ROS distribution and hardware setup still need to be recorded.

```bash
git clone https://github.com/Edipo3098/ros2_orange.git
cd ros2_orange
```

Before building, resolve the dependency and launch issues listed below. Once the workspace configuration is repaired, the standard build sequence is:

```bash
# Source your installed ROS 2 environment first.
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

These commands are a build outline, not a verified clean-install recipe. Hardware nodes may open device files immediately.

## Current limitations

- The README and execution instructions are being reconstructed from source; physical hardware and simulation have not been validated during this documentation update.
- Robot URDF models are included in `src/quadruped_arm_motion/urdf/`: `quadruped_arm.urdf`, `quadruped_arm_robot_control_real_effort.urdf` and `quadruped_arm_robot_control_real_effortTest_develope.urdf`. The specific filename referenced by `simulation_launch.py` is different (`quadruped_arm_robot_Control_effort.urdf`); this launch-to-model reference still needs reconciliation.
- `.gitmodules` contains stale `src/src/...` paths, while the `src/teleop` gitlink has no matching entry. Reconcile the dependency list before recursive submodule initialization.
- Serial settings such as `/dev/ttyS5` are embedded in source. They must match the board and controller protocol.
- Several scripts are experimental variants. Their names alone do not establish which implementation is the final one.
- Package metadata still contains license placeholders. Preserve existing third-party notices and resolve licensing before redistribution.

## Demonstration and project context

This repository belongs to the quadruped-with-manipulator TFM project. The project reference and supporting material are available through the [official UPC publication record](https://upcommons.upc.edu/entities/publication/f53c69dd-fdc5-4073-99e1-0c4ce5c5da53).

The publication link was supplied by the project author. Its contents could not be retrieved during this update because UPCommons returned HTTP 403; no title, numerical results or direct video URL have been inferred.

Related hardware experiments: [Orange_repo](https://github.com/Edipo3098/Orange_repo).

## Development priorities

1. Record the validated board, OS, ROS distribution and final runtime configuration.
2. Repair dependency declarations and provide one launch command for visualization without hardware.
3. Add a short demonstration and document the message flow and controller protocol.
4. Parameterize device settings and improve serial error handling.
5. Add functional tests for message conversion, communication failures and kinematics.

The current package test directories primarily contain style and documentation checks; hardware behavior needs separate validation.
