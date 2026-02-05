# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 Humble workspace for physical AI robotics development, currently focused on the Neuromeka Indy robot series. The workspace follows standard ROS2 colcon workspace structure.

**Workspace Name:** pipet_physical_ai_ws
**ROS2 Distribution:** Humble
**DDS Implementation:** Cyclone DDS (must be set via `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)

## Common Commands

### Building

```bash
# Build entire workspace from workspace root
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with symlink install (useful for Python development)
colcon build --symlink-install
```

### Sourcing

```bash
# Must be run from workspace root before using ROS2 commands
. install/setup.bash
```

### Direct Teaching / Data Collection

```bash
# Launch direct teaching mode with real robot
ros2 launch indy7_gripper_teleop direct_teaching_with_driver.launch.py \
  indy_ip:=192.168.0.6 \
  indy_type:=indy7 \
  data_dir:=~/teaching_data

# Launch direct teaching node only (assumes indy_driver is running)
ros2 launch indy7_gripper_teleop direct_teaching.launch.py

# Controls in teaching mode:
#   SPACE - Start/stop recording episode
#   Q     - Quit
#   R     - Reset episode counter
#   H     - Move to home position
```

### Launching

**Simulation with Gazebo:**
```bash
# Basic robot visualization
ros2 launch indy_description indy_display.launch.py indy_type:=indy7

# Gazebo simulation
ros2 launch indy_gazebo indy_gazebo.launch.py indy_type:=<robot_type>

# MoveIt with Gazebo
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=<robot_type>

# MoveIt with servoing mode
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=<robot_type> servo_mode:=true
```

**Real Robot:**
```bash
# Basic driver (replace IP with actual robot IP)
ros2 launch indy_driver indy_bringup.launch.py indy_type:=<robot_type> indy_ip:=192.168.xxx.xxx

# MoveIt with real robot
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=<robot_type> indy_ip:=192.168.xxx.xxx

# MoveIt with servoing on real robot
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=<robot_type> indy_ip:=192.168.xxx.xxx servo_mode:=true
```

**Teleop Control:**
```bash
# Keyboard control (use is_sim:=true for simulation, is_sim:=false for real robot)
ros2 run indy_driver servo_keyboard_input.py --ros-args -p is_sim:=<true/false>

# Joystick/gamepad control
ros2 run indy_driver servo_joy_input.py --ros-args -p is_sim:=<true/false>
```

### URDF Generation

```bash
# Generate URDF files from xacro templates
cd src/hardware_drivers/indy-ros2/indy_description/urdf/
sudo chmod +x generate_all_urdfs.sh
./generate_all_urdfs.sh
```

## Architecture

### Package Structure

The workspace is organized into two main categories:

**Hardware Drivers (`src/hardware_drivers/indy-ros2/`):**
- `indy_driver`: Core ROS2 driver node that communicates with the robot using the Neuromeka Python SDK
- `indy_interfaces`: Custom ROS2 messages, services, and actions
- `indy_description`: URDF/xacro robot models and visualization configurations
- `indy_gazebo`: Gazebo simulation launch files and world configurations
- `indy_moveit`: MoveIt2 motion planning configurations

**Teleop Control (`src/teleop_control/`):**
- `indy7_gripper_teleop`: Direct teaching mode and data collection for imitation learning
  - Enables gravity compensation mode for physical arm manipulation
  - Logs joint states at 20 Hz during demonstrations
  - Saves data in multiple formats (pickle, numpy, JSON) for training
  - Future: Will integrate gripper (Mark 7) and camera (RealSense D435)

### Key Components

**indy_driver Package:**
- Main driver node: [indy_driver.py](src/hardware_drivers/indy-ros2/indy_driver/indy_driver/indy_driver.py)
- Uses `neuromeka` Python package (`IndyDCP3` and `EtherCAT` classes) to communicate with robot
- Implements `FollowJointTrajectory` action server for trajectory execution
- Publishes `JointState` messages at 20 Hz
- Provides `IndyService` for direct robot commands
- Handles servo data transmission (ServoTx, ServoRx) for EtherCAT communication

**indy_interfaces Package:**
- Custom messages: `ServoTx.msg`, `ServoRx.msg`, `ServoDataArray.msg`
- Custom service: `IndyService.srv`
- These are ROS2 interface definitions that must be built before other packages can use them

**Robot Model (indy_description):**
- Uses xacro for parametric URDF generation
- Supports multiple robot types via configuration
- Includes ros2_control definitions for hardware interfaces
- Robot meshes and visual properties defined separately for each model

### Supported Robot Types

The system supports multiple Indy robot variants (specified via `indy_type` parameter):
- `indy7` (default)
- `indy7_v2`
- `indy12`
- `indy12_v2`
- `indyrp2`
- `indyrp2_v2`

Additional models in config: icon3, icon7l, nuri series, opti5

**Indy Eye:** Some models support "Indy Eye" vision system. Enable with `indy_eye:=true` parameter (supported on indy7, indy7_v2, indyrp2, indyrp2_v2).

### MoveIt Integration

The `indy_moveit` package contains robot-specific MoveIt configurations in subdirectories under `config/`. Each robot type has its own:
- SRDF (semantic robot description)
- Joint limits configuration
- Kinematics solver configuration (default uses KDL)
- Motion planning parameters

## Important Development Notes

### Dependencies

**Required System Packages:**
- ROS2 Humble
- Cyclone DDS: `ros-humble-rmw-cyclonedds-cpp`
- MoveIt2: `ros-humble-moveit` and related packages
- ros2_control: `ros-humble-ros2-control`, `ros-humble-ros2-controllers`
- Gazebo: `ros-humble-gazebo-ros`, `ros-humble-gazebo-ros2-control`
- **Message generation:** `ros-humble-rosidl-default-generators` (required for building custom messages)

**Python Dependencies:**
- `neuromeka` package: Must be installed via pip3 (`pip3 install neuromeka`)
- NumPy (for data logging): `pip3 install numpy`

**Installation:**
```bash
# Install message generators if not already installed
sudo apt-get install ros-humble-rosidl-default-generators

# Install Python dependencies
pip3 install neuromeka numpy
```

### Parameter Patterns

Most launch files accept these common parameters:
- `indy_type`: Robot model selection (default: indy7)
- `indy_ip`: Robot IP address (required for real robot, not used in simulation)
- `servo_mode`: Enable/disable servoing mode (default: false)
- `indy_eye`: Enable Indy Eye vision system (default: false)
- `is_sim`: Distinguish between simulation and real robot (used in Python nodes)

### Build Order

Due to message/service dependencies:
1. `indy_interfaces` must be built first (or build entire workspace)
2. Other packages depend on `indy_interfaces` being available
3. Use `colcon build` from workspace root to handle dependencies automatically

### Communication Architecture

- Robot driver publishes joint states to `/joint_states` topic
- MoveIt plans trajectories and sends them via `FollowJointTrajectory` action
- Direct robot control available through `IndyService` service interface
- Servo-level communication for advanced control via ServoTx/ServoRx messages
- All QoS profiles use RELIABLE reliability policy
