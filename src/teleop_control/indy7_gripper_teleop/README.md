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
- ✅ **Camera integration** (RealSense D435) with synchronized RGB + Depth
- 🔄 Future: Gripper integration (Mark 7)

## Installation

### Prerequisites

- ROS2 Humble
- Indy7 robot with `indy_driver` package installed
- Python 3.8+
- Neuromeka Python SDK: `pip3 install neuromeka`

### Camera Prerequisites (Optional)

For camera integration with RealSense D435:

```bash
# Install RealSense SDK and ROS2 package
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-camera

# Install camera dependencies
sudo apt install ros-humble-cv-bridge ros-humble-message-filters ros-humble-image-transport

# Install OpenCV for Python (for image resizing)
pip3 install opencv-python
```

### Build Package

```bash
cd ~/pipet_physical_ai_ws
colcon build --packages-select indy7_gripper_teleop
source install/setup.bash
```

## Quick Start (직접교시 빠른 시작)

### Step 0: 네트워크 설정 (최초 1회)

로봇과 컴퓨터를 USB 이더넷 어댑터로 연결한 경우:

```bash
# USB 이더넷 어댑터에 고정 IP 설정
sudo ip addr add 192.168.1.100/24 dev enx00e04c360046

# 로봇 연결 확인
ping 192.168.1.10
```

### Step 1: Terminal 1 - ROS2 환경 설정 및 드라이버 실행

```bash
# ROS2 환경 설정
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Indy7 드라이버 실행
ros2 launch indy_driver indy_bringup.launch.py \
  indy_type:=indy7 \
  indy_ip:=192.168.1.10
```

### Step 2: Terminal 2 - 직접교시 노드 실행

```bash
# ROS2 환경 설정
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 직접교시 노드 실행 (auto_enable_teaching=false 권장)
ros2 run indy7_gripper_teleop teaching_control.py \
  --ros-args \
  -p data_dir:=~/teaching_data \
  -p auto_enable_teaching:=false
```

### Step 3: 직접교시 사용

1. **[H] 키** - 로봇을 홈 위치로 이동 (관절 한계 에러 방지)
2. **[S] 키** - 현재 로봇 상태 확인 (관절 위치, 한계값 근접 경고)
3. **[SPACE] 키** - 녹화 시작 (직접교시 모드 자동 활성화)
4. **로봇 팔을 손으로 움직여서 시연**
5. **[SPACE] 키** - 녹화 중지 및 저장
6. **[E] 키** - 에러 발생 시 복구 (빨간 LED 시)
7. **[Q] 키** - 종료

---

## Camera Integration (카메라 통합)

### 카메라 모드로 직접교시 실행

RealSense D435 카메라와 함께 로봇 데이터를 동기화하여 기록합니다.

**중요: 실행 순서를 반드시 지켜야 합니다!**

```bash
# Terminal 1: 로봇 드라이버 (먼저 실행)
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch indy_driver indy_bringup.launch.py indy_type:=indy7 indy_ip:=192.168.1.10

# Terminal 2: RealSense 카메라 (두 번째 실행)
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true

# Terminal 3: 직접교시 노드 (카메라가 실행된 후 마지막에 실행)
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run indy7_gripper_teleop teaching_control.py \
  --ros-args \
  -p data_dir:=~/teaching_data \
  -p enable_camera:=true \
  -p enable_depth:=true \
  -p resize_images:=true
```

**카메라 토픽 확인:**
```bash
# 카메라 토픽이 발행되고 있는지 확인
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

### Camera Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_camera` | `false` | Enable camera recording |
| `enable_depth` | `true` | Record depth images |
| `resize_images` | `true` | Resize to 224x224 for training |
| `camera_fps` | `15` | Camera frame rate |

### 카메라 데이터 형식

카메라 모드 활성화 시 저장되는 데이터:

```python
{
    'timestamps': np.array([...]),           # 동기화된 타임스탬프
    'joint_positions': np.array([...]),      # 관절 각도 (radians)
    'joint_velocities': np.array([...]),     # 관절 속도 (rad/s)
    'joint_efforts': np.array([...]),        # 관절 토크 (N⋅m)
    'rgb_images': np.array([...]),           # RGB 이미지 (N, 224, 224, 3)
    'depth_images': np.array([...]),         # Depth 이미지 (N, 224, 224), uint16
}
```

### 카메라 상태 확인

녹화 중 **[V] 키**를 눌러 카메라 동기화 상태를 확인할 수 있습니다:

```
  CAMERA SYNC STATUS
------------------------------------------------------------
  Recording: YES
  Sync Callbacks: 150
  Current Samples: 150
  RGB Frames: 150
  Depth Frames: 150
  Effective Rate: 15.0 Hz
------------------------------------------------------------
```

---

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
| `H` | Move robot to home position |
| `S` | Show robot status (joint positions, limits) |
| `E` | Error recovery (use when robot has red LED) |
| `V` | Show camera/sync status (when camera enabled) |
| `R` | Reset episode counter |
| `C` | Clear screen and show instructions |
| `Q` | Quit and disable teaching mode |

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

3. **CameraDataLogger** ([camera_data_logger.py](indy7_gripper_teleop/camera_data_logger.py))
   - Synchronized joint + camera data collection
   - Uses `ApproximateTimeSynchronizer` for multi-sensor alignment
   - Records RGB and depth images at ~15 Hz
   - Resizes images to 224x224 for imitation learning

4. **DirectTeachingNode** ([direct_teaching_node.py](indy7_gripper_teleop/direct_teaching_node.py))
   - Main coordination node
   - Keyboard input handling
   - Status publishing

### Topics

**Subscribed (Joint-only mode):**
- `/joint_states` (sensor_msgs/JointState) - Robot joint feedback

**Subscribed (Camera mode):**
- `/joint_states` (sensor_msgs/JointState) - Robot joint feedback
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - RGB image
- `/camera/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image) - Aligned depth

> **Note:** The camera topics have double namespace (`/camera/camera/...`) because the RealSense node runs with namespace `camera` and node name `camera`.

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

### 네트워크 연결 실패 (grpc UNAVAILABLE)

**에러 메시지:**
```
grpc._channel._InactiveRpcError: failed to connect to all addresses
```

**해결 방법:**
1. 로봇 전원이 켜져 있는지 확인
2. USB 이더넷 어댑터 IP 설정:
   ```bash
   sudo ip addr add 192.168.1.100/24 dev enx00e04c360046
   ```
3. 로봇 ping 테스트: `ping 192.168.1.10`
4. Conty 앱에서 로봇 상태 확인 (녹색 = 정상)

### 관절 한계 에러 (Joint Position Close To Limit)

**증상:** 로봇에 빨간 불이 들어오고 "Stop Category2" 에러

**해결 방법:**
1. Conty 앱에서 에러 리셋
2. `auto_enable_teaching:=false`로 노드 실행
3. [H] 키로 홈 위치 이동 먼저 실행
4. 그 후 [SPACE]로 녹화 시작

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

### Camera mode: 0 samples recorded

**Symptoms:** Recording shows "0 sync frames, 0.0 Hz"

**Causes & Solutions:**

1. **Wrong execution order:** Camera must be running BEFORE teaching node
   ```bash
   # Correct order:
   # 1. Robot driver → 2. Camera → 3. Teaching node
   ```

2. **Wrong topic names:** Check actual camera topic names
   ```bash
   ros2 topic list | grep camera
   # Should see /camera/camera/color/image_raw (double namespace)
   ```

3. **Camera not publishing:** Check camera Hz
   ```bash
   ros2 topic hz /camera/camera/color/image_raw
   # Should show ~15-30 Hz
   ```

### Camera USB 2.0 warning

**Warning:** "Device USB type: 2.1 - Reduced performance is expected"

**Solution:** Connect camera directly to USB 3.0 port (blue port) instead of through a hub.

### Verifying camera data was recorded

```python
import numpy as np
data = np.load('~/teaching_data/episode_XXX_camera.npz')
print("Keys:", list(data.keys()))
print("RGB images:", data['rgb_images'].shape)  # Should be (N, 224, 224, 3)
print("Depth images:", data['depth_images'].shape)  # Should be (N, 224, 224)
```

## Future Extensions

### Phase 2: Gripper Integration (Mark 7)

Will add:
- Gripper status subscription
- Synchronized gripper position logging
- Finger control during demonstration

### ~~Phase 3: Camera Integration (RealSense D435)~~ ✅ COMPLETED

Implemented features:
- ✅ RGB image capture (224x224 for training)
- ✅ Depth image capture (aligned to color)
- ✅ Multi-sensor synchronization using `ApproximateTimeSynchronizer`
- ✅ Vision-based learning support with synchronized timestamps

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

**Note:** This implementation supports Indy7 arm with RealSense D435 camera integration. Gripper integration (Mark 7) will be added in future updates.

**한글 문서:** [README_KR.md](README_KR.md)
