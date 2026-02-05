# Indy7 Direct Teaching Teleoperation

Direct teaching mode and data collection package for the Indy7 robot, designed for imitation learning and physical AI applications.

## Overview

This package enables **direct teaching mode** on the Indy7 robot, allowing users to physically move the robot arm while automatically logging demonstration data. The collected data can be used for:

- Behavior cloning / Imitation learning
- Diffusion policy training
- Reinforcement learning with demonstrations
- Digital twin training in Isaac Sim

## Features

- ✅ Direct teaching mode with gravity compensation
- ✅ Real-time joint state logging at 20 Hz
- ✅ Keyboard-based episode recording control
- ✅ Multiple data formats (pickle, numpy, JSON metadata)
- ✅ Automatic episode management
- ✅ Safety monitoring (collision detection, joint limits)
- 🔄 Future: Gripper integration (Mark 7)
- 🔄 Future: Camera integration (RealSense D435)

## Installation

### Prerequisites

- ROS2 Humble
- Indy7 robot with `indy_driver` package installed
- Python 3.8+
- Neuromeka Python SDK: `pip3 install neuromeka`

### Build Package

```bash
cd ~/pipet_physical_ai_ws
colcon build --packages-select indy7_gripper_teleop
source install/setup.bash
```

## Usage

### Option 1: Launch with Indy Driver (Recommended for Real Robot)

This launches both the indy_driver and direct teaching node:

```bash
ros2 launch indy7_gripper_teleop direct_teaching_with_driver.launch.py \
  indy_ip:=192.168.0.6 \
  indy_type:=indy7 \
  data_dir:=~/teaching_data
```

**Parameters:**
- `indy_ip` - IP address of the Indy7 robot (required for real robot)
- `indy_type` - Robot type (default: `indy7`)
- `data_dir` - Directory to save demonstration data (default: `~/teaching_data`)
- `auto_enable_teaching` - Auto-enable teaching mode on startup (default: `true`)

### Option 2: Launch Standalone (Assumes Driver is Already Running)

If `indy_driver` is already running:

```bash
ros2 launch indy7_gripper_teleop direct_teaching.launch.py \
  data_dir:=~/teaching_data
```

### Keyboard Controls

Once the node is running, you can control recording with:

| Key | Action |
|-----|--------|
| `SPACE` | Start/Stop recording episode |
| `Q` | Quit and disable teaching mode |
| `R` | Reset episode counter |
| `H` | Move robot to home position |
| `C` | Clear screen and show instructions |

### Typical Workflow

1. **Launch the system** using one of the launch commands above
2. **Verify teaching mode is enabled** - you should be able to physically move the robot arm
3. **Position the robot** at the starting configuration for your demonstration
4. **Press SPACE** to start recording
5. **Perform the demonstration** by physically moving the robot
6. **Press SPACE** again to stop recording and save the episode
7. **Repeat steps 3-6** for multiple demonstrations
8. **Press Q** to quit when done

## Data Format

### File Structure

Each episode is saved in three formats:

```
~/teaching_data/
├── episode_001_20260204_153045.pkl           # Python pickle format
├── episode_001_20260204_153045.npz           # NumPy arrays
├── episode_001_20260204_153045_metadata.json # JSON metadata
├── episode_002_20260204_153120.pkl
└── ...
```

### Data Contents

**Pickle/NumPy format** contains:
```python
{
    'timestamps': np.array([...]),          # Relative timestamps (seconds)
    'joint_positions': np.array([[...], ...]),  # Joint angles (radians)
    'joint_velocities': np.array([[...], ...]), # Joint velocities (rad/s)
    'joint_efforts': np.array([[...], ...]),    # Joint torques (N⋅m)
    'joint_names': ['joint0', 'joint1', ...]    # Joint names
}
```

**JSON metadata** contains:
```json
{
    "episode_id": 1,
    "duration": 30.5,
    "sample_count": 610,
    "sample_rate": 20.0,
    "robot_type": "indy7",
    "timestamp": "2026-02-04T15:30:45",
    "joint_names": ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
}
```

### Loading Data in Python

```python
import pickle
import numpy as np

# Load pickle format
with open('episode_001_20260204_153045.pkl', 'rb') as f:
    data = pickle.load(f)

# Or load NumPy format
data = np.load('episode_001_20260204_153045.npz')
positions = data['joint_positions']
velocities = data['joint_velocities']
```

## Architecture

### Components

1. **TeachingModeManager** ([teaching_mode_manager.py](indy7_gripper_teleop/teaching_mode_manager.py))
   - Manages robot teaching mode activation/deactivation
   - Interfaces with `indy_srv` service
   - Handles recovery and home position commands

2. **DataLogger** ([data_logger.py](indy7_gripper_teleop/data_logger.py))
   - Subscribes to `/joint_states` topic
   - Collects data at 20 Hz during recording
   - Saves episodes in multiple formats

3. **DirectTeachingNode** ([direct_teaching_node.py](indy7_gripper_teleop/direct_teaching_node.py))
   - Main coordination node
   - Keyboard input handling
   - Status publishing

### Topics

**Subscribed:**
- `/joint_states` (sensor_msgs/JointState) - Robot joint feedback

**Published:**
- `/teaching_status` (std_msgs/String) - Current recording status

**Services:**
- `/indy_srv` (indy_interfaces/IndyService) - Robot mode control

## Configuration

### Teaching Parameters

Edit [config/teaching_params.yaml](config/teaching_params.yaml) to customize:
- Data directory location
- Auto-enable teaching mode
- Joint limits
- Safety parameters

### Data Collection Config

Edit [config/data_collection_config.yaml](config/data_collection_config.yaml) to customize:
- Data save formats
- Topic names
- Expected frequencies

## Troubleshooting

### Robot is not movable after launching

**Issue:** Teaching mode may not be properly enabled.

**Solution:**
1. Check that `indy_driver` is running: `ros2 node list | grep indy_driver`
2. Check robot state: `ros2 topic echo /joint_states`
3. Manually enable teaching mode by stopping teleop:
   ```bash
   ros2 service call /indy_srv indy_interfaces/srv/IndyService "{data: 8}"
   ```
4. **Note:** Current implementation uses MSG_TELE_STOP. If robot still not movable, compliance mode API may need to be explicitly called. See TODO in [teaching_mode_manager.py:56](indy7_gripper_teleop/teaching_mode_manager.py#L56).

### Data is not being recorded

**Possible causes:**
- `/joint_states` topic not publishing - check with `ros2 topic hz /joint_states`
- Recording not started - press SPACE to start recording
- File permissions - check write access to data directory

### Low sampling rate

**Expected:** 20 Hz (same as joint_states publication rate)

**If lower:**
- Check system load: `htop`
- Verify joint_states frequency: `ros2 topic hz /joint_states`
- Reduce file I/O by saving less frequently

## Future Extensions

### Phase 2: Gripper Integration (Mark 7)

Will add:
- Gripper status subscription
- Synchronized gripper position logging
- Finger control during demonstration

### Phase 3: Camera Integration (RealSense D435)

Will add:
- RGB image capture
- Depth image capture
- Multi-sensor synchronization using `ApproximateTimeSynchronizer`
- Vision-based learning support

### Phase 4: Isaac Sim Digital Twin

Will add:
- Real-time trajectory mirroring to Isaac Sim
- Bidirectional communication
- Sim-to-real validation

## Related Documentation

- Main workspace: [CLAUDE.md](../../../CLAUDE.md)
- Project structure: [docs/structure.md](../../../docs/structure.md)
- Indy Driver: [src/hardware_drivers/indy-ros2/README.md](../../hardware_drivers/indy-ros2/README.md)

## License

BSD-3-Clause

## Maintainer

sirlab

---

**Note:** This is an initial implementation focusing on Indy7 arm only. Gripper and camera integration will be added in future updates.
