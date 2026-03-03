# Pipet Physical AI 프로젝트 기술 문서

**직접 교시 기반 피지컬 AI 데이터 수집 및 모방학습 시스템**

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [시스템 구성](#2-시스템-구성)
3. [워크스페이스 구조](#3-워크스페이스-구조)
4. [실행 방법](#4-실행-방법)
5. [데이터 수집 파이프라인](#5-데이터-수집-파이프라인)
6. [수집 데이터 구조](#6-수집-데이터-구조)
7. [모방학습 프레임워크 연동](#7-모방학습-프레임워크-연동)
8. [향후 로드맵](#8-향후-로드맵)

---

## 1. 프로젝트 개요

### 1.1 목표

실제 로봇(Indy7 + Mark 7 그리퍼)을 직접 움직여 얻은 시연 데이터를 바탕으로, 모방학습(Imitation Learning) 및 강화학습(Reinforcement Learning)을 수행하여 정밀한 파이펫팅(Pipetting) 지능을 구현한다.

### 1.2 핵심 기술 스택

| 구분 | 기술 |
|------|------|
| **Hardware** | Neuromeka Indy7 (로봇 팔), Mand.ro Mark 7 (그리퍼), Intel RealSense D435 (카메라) |
| **Middleware** | ROS2 Humble, Cyclone DDS |
| **시뮬레이션** | NVIDIA Isaac Sim + Isaac Lab, Unity ML-Agents |
| **학습 프레임워크** | Robomimic, LeRobot, Diffusion Policy, ACT |
| **알고리즘** | Behavior Cloning, Diffusion Policy, PPO, GAIL |

### 1.3 전체 파이프라인

```
┌─────────────────────────────────────────────────────────────────┐
│                    데이터 수집 (Real World)                       │
│                                                                  │
│   사람이 로봇 팔을 직접 움직임 (직접 교시)                           │
│   → 관절 상태 + RGB + Depth 동기화 기록 (~15 Hz)                   │
│   → NPZ/PKL/JSON 형식으로 저장                                    │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    데이터 변환 (Conversion)                       │
│                                                                  │
│   NPZ → HDF5 (Robomimic)                                        │
│   NPZ → Parquet + MP4 (LeRobot)                                 │
│   NPZ → .demo (Unity ML-Agents)                                 │
└──────────────────────┬──────────────────────────────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
┌──────────────┐ ┌──────────┐ ┌──────────────┐
│  Isaac Lab   │ │  LeRobot │ │ Unity ML-Agents│
│  + Robomimic │ │          │ │              │
│              │ │  ACT     │ │  GAIL + PPO  │
│  BC → PPO   │ │  Diffusion│ │  BC          │
│  (1024 envs)│ │  Policy  │ │              │
└──────┬───────┘ └────┬─────┘ └──────┬───────┘
       │              │              │
       └──────────────┼──────────────┘
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    배포 (Deployment)                              │
│                                                                  │
│   학습된 모델 → ROS2 노드로 래핑                                   │
│   → 시뮬레이션 검증 (Isaac Sim)                                   │
│   → 실제 로봇 배포 (Sim-to-Real)                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 시스템 구성

### 2.1 하드웨어

#### 로봇 팔 - Neuromeka Indy7

| 항목 | 사양 |
|------|------|
| 자유도 | 6 DOF |
| 통신 | gRPC (IndyDCP3 프로토콜) |
| 제어 주기 | 20 Hz (ROS2 joint_states) |
| 직접 교시 | 중력 보상 모드 지원 |
| IP 주소 | 192.168.1.10 (USB 이더넷) |

#### 카메라 - Intel RealSense D435

| 항목 | 사양 |
|------|------|
| RGB 해상도 | 640×480 → 224×224 (리사이즈) |
| Depth 해상도 | 640×480 → 224×224 (리사이즈) |
| 프레임 레이트 | 15~30 Hz |
| 인터페이스 | USB 3.0 (USB 2.0에서는 성능 저하) |
| Depth 정렬 | Aligned Depth to Color 사용 |

#### 그리퍼 - Mand.ro Mark 7 (예정)

| 항목 | 사양 |
|------|------|
| 통신 | 115200bps 시리얼 |
| 제어 패킷 | 15 byte 전송 |
| 상태 패킷 | 19 byte 수신 |
| 기능 | 개별 손가락 정밀 위치 제어 |
| 제어 방식 | 직접교시 중 키보드로 3가지 행동 제어 |

**그리퍼 행동 정의:**

| 행동 | 설명 | 용도 |
|------|------|------|
| **잡기 (Grasp)** | 손가락을 오므려 대상을 파지 | 파이펫을 잡는 동작 |
| **펴기 (Open)** | 손가락을 펼쳐 대상을 놓음 | 파이펫을 놓는 동작 |
| **파이펫 누르기 (Press)** | 엄지로 파이펫 버튼을 누름 | 파이펫팅 실행 동작 |

직접교시 중 사용자가 한 손으로 로봇 팔을 움직이면서, 다른 손으로 키보드를 눌러 그리퍼 행동을 제어합니다. 그리퍼 행동은 이산(discrete) 액션으로 기록됩니다.

### 2.2 소프트웨어 환경

```bash
# 필수 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/Dev/ROS2/pipet_physical_ai_ws/install/setup.bash
```

| 패키지 | 용도 |
|--------|------|
| `ros-humble-rmw-cyclonedds-cpp` | DDS 통신 |
| `ros-humble-realsense2-camera` | RealSense 카메라 드라이버 |
| `ros-humble-cv-bridge` | ROS Image ↔ OpenCV 변환 |
| `ros-humble-message-filters` | 다중 토픽 시간 동기화 |
| `neuromeka` (pip) | Indy7 로봇 SDK |
| `opencv-python` (pip) | 이미지 리사이징 |
| `numpy` (pip) | 데이터 저장/처리 |

---

## 3. 워크스페이스 구조

```
pipet_physical_ai_ws/
├── src/
│   ├── hardware_drivers/
│   │   └── indy-ros2/                    # Neuromeka 공식 ROS2 드라이버 (서브모듈)
│   │       ├── indy_driver/              # 로봇 제어 노드
│   │       │   └── indy_driver/
│   │       │       ├── indy_driver.py    # 메인 드라이버 (JointState 퍼블리셔)
│   │       │       └── indy_define.py    # 메시지 코드 정의
│   │       ├── indy_interfaces/          # 커스텀 메시지/서비스 정의
│   │       ├── indy_description/         # URDF/xacro 로봇 모델
│   │       ├── indy_gazebo/              # Gazebo 시뮬레이션
│   │       └── indy_moveit/              # MoveIt2 설정
│   │
│   └── teleop_control/
│       └── indy7_gripper_teleop/         # ★ 직접교시 패키지
│           ├── indy7_gripper_teleop/
│           │   ├── __init__.py
│           │   ├── direct_teaching_node.py    # 메인 노드 (키보드 제어)
│           │   ├── teaching_mode_manager.py   # 직접교시 모드 관리
│           │   ├── data_logger.py             # 관절 데이터 로거 (20 Hz)
│           │   └── camera_data_logger.py      # 카메라+관절 동기화 로거 (~15 Hz)
│           ├── launch/
│           │   ├── direct_teaching.launch.py
│           │   ├── direct_teaching_with_driver.launch.py
│           │   └── direct_teaching_with_camera.launch.py
│           ├── config/
│           │   ├── teaching_params.yaml
│           │   └── data_collection_config.yaml
│           └── scripts/
│               └── teaching_control.py        # 실행 스크립트
│
├── docs/
│   ├── structure.md                      # 프로젝트 기획서
│   └── pipet_physical_ai_project.md      # ★ 이 문서
│
├── install/                              # colcon 빌드 결과
├── build/
├── log/
└── CLAUDE.md                             # AI 어시스턴트 가이드
```

### 3.1 주요 컴포넌트 관계

```
┌──────────────────────┐
│   DirectTeachingNode │  ← 키보드 입력 처리, 전체 조정
│   (direct_teaching_  │
│    node.py)          │
└──────┬───────┬───────┘
       │       │
       ▼       ▼
┌──────────┐ ┌──────────────────┐
│ Teaching │ │ CameraDataLogger │  ← 카메라 모드 (enable_camera:=true)
│ Mode     │ │ (camera_data_    │
│ Manager  │ │  logger.py)      │
└──────────┘ └──────────────────┘
     │              │
     ▼              ▼
┌──────────┐ ┌──────────────────────────────────────────┐
│ indy_srv │ │ ApproximateTimeSynchronizer               │
│ service  │ │   ├─ /joint_states (20 Hz)               │
│          │ │   ├─ /camera/camera/color/image_raw       │
└──────────┘ │   └─ /camera/camera/aligned_depth_to_    │
             │      color/image_raw                      │
             └──────────────────────────────────────────┘
```

---

## 4. 실행 방법

### 4.1 사전 준비

```bash
# 1. 네트워크 설정 (USB 이더넷 어댑터)
sudo ip addr add 192.168.1.100/24 dev enx00e04c360046

# 2. 로봇 연결 확인
ping 192.168.1.10

# 3. 카메라 연결 확인 (USB 3.0 포트에 직접 연결 권장)
lsusb | grep Intel
# Bus 001 Device 003: ID 8086:0b07 Intel Corp. RealSense D435
```

### 4.2 관절 데이터만 수집 (카메라 없이)

```bash
# Terminal 1: 로봇 드라이버
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch indy_driver indy_bringup.launch.py indy_type:=indy7 indy_ip:=192.168.1.10

# Terminal 2: 직접교시 노드
cd ~/Dev/ROS2/pipet_physical_ai_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run indy7_gripper_teleop teaching_control.py \
  --ros-args -p data_dir:=~/teaching_data
```

### 4.3 카메라 + 관절 데이터 동시 수집

**중요: 반드시 아래 순서대로 실행해야 합니다!**

```bash
# Terminal 1: 로봇 드라이버 (먼저 실행)
ros2 launch indy_driver indy_bringup.launch.py indy_type:=indy7 indy_ip:=192.168.1.10

# Terminal 2: RealSense 카메라 (두 번째 실행)
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true

# Terminal 3: 직접교시 노드 (카메라 발행 확인 후 마지막 실행)
ros2 run indy7_gripper_teleop teaching_control.py \
  --ros-args \
  -p data_dir:=~/teaching_data \
  -p enable_camera:=true \
  -p enable_depth:=true \
  -p resize_images:=true
```

### 4.4 키보드 조작

**시스템 제어:**

| 키 | 동작 |
|----|------|
| `SPACE` | 에피소드 녹화 시작/중지 |
| `H` | 로봇을 홈 위치로 이동 |
| `S` | 로봇 상태 표시 (관절 위치, 한계) |
| `E` | 에러 복구 (빨간 LED 시 사용) |
| `V` | 카메라/동기화 상태 표시 |
| `R` | 에피소드 카운터 리셋 |
| `C` | 화면 지우고 안내 표시 |
| `Q` | 종료 |

**그리퍼 제어 (Phase 2 예정):**

| 키 | 행동 | 설명 |
|----|------|------|
| `G` | 잡기 (Grasp) | 손가락을 오므려 파이펫 파지 |
| `O` | 펴기 (Open) | 손가락을 펼쳐 파이펫 놓기 |
| `P` | 파이펫 누르기 (Press) | 엄지로 파이펫 버튼 누름 |

> 직접교시 중 한 손으로 로봇 팔을 움직이면서, 다른 손으로 키보드를 눌러 그리퍼를 제어합니다.

### 4.5 일반적인 워크플로우

1. 로봇 드라이버 + 카메라 실행
2. 직접교시 노드 실행
3. `[H]` 로봇을 홈 위치로 이동
4. `[SPACE]` 녹화 시작
5. 로봇 팔을 한 손으로 잡고 시연 동작 수행
6. 다른 손으로 키보드를 눌러 그리퍼 제어 (Phase 2: `[G]` 잡기, `[O]` 펴기, `[P]` 파이펫 누르기)
7. `[SPACE]` 녹화 중지 (자동 저장)
8. 3~7 반복하여 다수의 에피소드 수집
9. `[Q]` 종료

---

## 5. 데이터 수집 파이프라인

### 5.1 동기화 메커니즘

`ApproximateTimeSynchronizer`를 사용하여 3개 토픽의 메시지를 시간 기준으로 정렬합니다.

```
/joint_states (20 Hz)  ──┐
                          ├── ApproximateTimeSynchronizer (slop=0.1s)
/camera/.../image_raw  ──┤                    │
                          │                    ▼
/camera/.../depth_raw  ──┘          동기화된 콜백 (~15 Hz)
                                       │
                                       ▼
                              ┌─────────────────┐
                              │  데이터 버퍼     │
                              │  - timestamps    │
                              │  - joint_pos     │
                              │  - joint_vel     │
                              │  - joint_eff     │
                              │  - rgb_images    │
                              │  - depth_images  │
                              └─────────────────┘
                                       │
                                  [SPACE] 누름
                                       │
                                       ▼
                              ┌─────────────────┐
                              │  파일 저장       │
                              │  - .pkl (원본)   │
                              │  - .npz (압축)   │
                              │  - .json (메타)  │
                              └─────────────────┘
```

### 5.2 토픽 구조

| 토픽 | 메시지 타입 | 주기 | 설명 |
|------|------------|------|------|
| `/joint_states` | sensor_msgs/JointState | 20 Hz | 6개 관절 (위치, 속도, 토크) |
| `/camera/camera/color/image_raw` | sensor_msgs/Image | 15~30 Hz | RGB 이미지 |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 15~30 Hz | Depth 이미지 (mm) |
| `/gripper/status` | (예정) | 20 Hz | 그리퍼 상태 (Phase 2) |
| `/gripper/command` | (예정) | 이벤트 | 그리퍼 행동 명령 (Phase 2) |

> **참고:** 카메라 토픽은 이중 네임스페이스 (`/camera/camera/...`)를 사용합니다.

### 5.3 이미지 처리

- **RGB**: `cv_bridge`로 변환 → `cv2.INTER_LINEAR`로 224×224 리사이즈
- **Depth**: `passthrough` 인코딩으로 변환 → `cv2.INTER_NEAREST`로 224×224 리사이즈 (값 보존)

---

## 6. 수집 데이터 구조

### 6.1 파일 구조

```
~/teaching_data/
├── episode_001_20260210_183213_camera.pkl           # Pickle (40 MB)
├── episode_001_20260210_183213_camera.npz           # NumPy 압축 (16 MB)
├── episode_001_20260210_183213_camera_metadata.json  # 메타데이터
├── episode_002_20260210_183345_camera.pkl
├── episode_002_20260210_183345_camera.npz
├── episode_002_20260210_183345_camera_metadata.json
└── ...
```

파일명 규칙: `episode_{ID:03d}_{YYYYMMDD_HHMMSS}_camera.{ext}`

### 6.2 NPZ 데이터 (학습에 주로 사용)

```python
import numpy as np
data = np.load('episode_001_20260210_183213_camera.npz')
```

| 키 | Shape | dtype | 단위 | 설명 |
|----|-------|-------|------|------|
| `timestamps` | (N,) | float64 | 초 | 녹화 시작 기준 상대 시간 |
| `joint_positions` | (N, 6) | float64 | radians | 6개 관절 각도 |
| `joint_velocities` | (N, 6) | float64 | rad/s | 6개 관절 속도 |
| `joint_efforts` | (N, 6) | float64 | N·m | 6개 관절 토크 |
| `rgb_images` | (N, 224, 224, 3) | uint8 | 0-255 | RGB 컬러 이미지 |
| `depth_images` | (N, 224, 224) | uint16 | mm | Depth 거리 이미지 |
| `gripper_actions` | (N,) | int8 | - | 그리퍼 행동 (Phase 2 예정) |

- **N**: 샘플 수 (예: 10초 녹화 시 ~150개)
- **동기화 레이트**: ~15 Hz (카메라 프레임 레이트에 의해 제한)

**그리퍼 행동 인코딩 (Phase 2 예정):**

| 값 | 행동 | 설명 |
|----|------|------|
| `0` | 유지 (Idle) | 현재 상태 유지 (대부분의 프레임) |
| `1` | 잡기 (Grasp) | 손가락 오므림 → 파이펫 파지 |
| `2` | 펴기 (Open) | 손가락 펼침 → 파이펫 놓기 |
| `3` | 파이펫 누르기 (Press) | 엄지로 파이펫 버튼 누름 |

> 키보드 입력 시점의 타임스탬프에 해당하는 프레임에 행동 값이 기록되며, 나머지 프레임은 `0`(유지)으로 채워집니다.

### 6.3 JSON 메타데이터

```json
{
  "episode_id": 1,
  "duration": 11.28,
  "sample_count": 168,
  "sample_rate": 14.9,
  "joint_names": ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"],
  "timestamp": "2026-02-10T18:32:13.456789",
  "has_rgb": true,
  "has_depth": true,
  "rgb_count": 168,
  "depth_count": 168,
  "image_size": [224, 224],
  "camera_enabled": true
}
```

### 6.4 데이터 로딩 예시

```python
import numpy as np
import pickle

# 방법 1: NumPy (권장 - 빠르고 효율적)
data = np.load('episode_001_camera.npz')
joint_pos = data['joint_positions']   # (168, 6)
rgb = data['rgb_images']              # (168, 224, 224, 3)
depth = data['depth_images']          # (168, 224, 224)

# 방법 2: Pickle (메타데이터 포함)
with open('episode_001_camera.pkl', 'rb') as f:
    data = pickle.load(f)
metadata = data['metadata']
joint_pos = data['joint_positions']
```

### 6.5 데이터 확인

```python
# 저장된 데이터 검증 스크립트
import numpy as np
data = np.load('episode_001_camera.npz')

print("포함된 키:", list(data.keys()))
for key in data.keys():
    print(f"  {key}: shape={data[key].shape}, dtype={data[key].dtype}")

# 예상 출력:
# 포함된 키: ['timestamps', 'joint_positions', 'joint_velocities', 'joint_efforts', 'rgb_images', 'depth_images']
#   timestamps: shape=(168,), dtype=float64
#   joint_positions: shape=(168, 6), dtype=float64
#   joint_velocities: shape=(168, 6), dtype=float64
#   joint_efforts: shape=(168, 6), dtype=float64
#   rgb_images: shape=(168, 224, 224, 3), dtype=uint8
#   depth_images: shape=(168, 224, 224), dtype=uint16
```

---

## 7. 모방학습 프레임워크 연동

수집된 NPZ 데이터를 세 가지 학습 프레임워크에서 사용하는 방법을 설명합니다.

### 7.1 NVIDIA Isaac Sim + Isaac Lab + Robomimic

**개요:** GPU 가속 물리 시뮬레이션으로 1024개 이상의 환경을 병렬 실행하며, Robomimic으로 모방학습 후 PPO로 강화학습 파인튜닝.

#### 데이터 변환: NPZ → HDF5 (Robomimic)

Robomimic은 아래 구조의 HDF5 파일을 요구합니다:

```
dataset.hdf5
├── data/
│   ├── demo_0/
│   │   ├── actions          (N, 6) float32    # 정규화된 관절 명령
│   │   ├── rewards          (N,)   float32
│   │   ├── dones            (N,)   int32
│   │   ├── obs/
│   │   │   ├── joint_pos    (N, 6) float32
│   │   │   ├── joint_vel    (N, 6) float32
│   │   │   ├── rgb          (N, 224, 224, 3) uint8
│   │   │   └── depth        (N, 224, 224) float32  # meters
│   │   └── next_obs/
│   │       ├── joint_pos    (N, 6) float32   # t+1 시점 관측
│   │       └── ...
│   ├── demo_1/
│   └── ...
└── mask/
    ├── train              ["demo_0", "demo_1", ...]
    └── valid              ["demo_5", ...]
```

**변환 코드:**

```python
import h5py
import numpy as np
import glob, json

def npz_to_robomimic_hdf5(npz_dir, output_path):
    """NPZ 에피소드를 Robomimic HDF5로 변환"""
    npz_files = sorted(glob.glob(f"{npz_dir}/episode_*_camera.npz"))

    with h5py.File(output_path, "w") as f:
        data_grp = f.create_group("data")
        data_grp.attrs["env_args"] = json.dumps({
            "env_name": "Indy7DirectTeaching",
            "env_type": 2,
            "env_kwargs": {"robot_type": "indy7", "control_freq": 15}
        })

        for ep_idx, npz_file in enumerate(npz_files):
            data = np.load(npz_file)
            joint_pos = data["joint_positions"].astype(np.float32)
            joint_vel = data["joint_velocities"].astype(np.float32)
            rgb = data["rgb_images"]
            depth = data["depth_images"].astype(np.float32) / 1000.0  # mm → m

            N = joint_pos.shape[0]

            # Action: 다음 시점의 관절 위치 (absolute)
            actions = np.concatenate([joint_pos[1:], joint_pos[-1:]], axis=0)

            demo = data_grp.create_group(f"demo_{ep_idx}")
            demo.attrs["num_samples"] = N
            demo.create_dataset("actions", data=actions)
            demo.create_dataset("rewards", data=np.zeros(N, dtype=np.float32))
            dones = np.zeros(N, dtype=np.int32); dones[-1] = 1
            demo.create_dataset("dones", data=dones)

            obs = demo.create_group("obs")
            obs.create_dataset("joint_pos", data=joint_pos)
            obs.create_dataset("joint_vel", data=joint_vel)
            obs.create_dataset("rgb", data=rgb)
            obs.create_dataset("depth", data=depth)

            # next_obs: 한 타임스텝 이동
            next_obs = demo.create_group("next_obs")
            next_obs.create_dataset("joint_pos",
                data=np.concatenate([joint_pos[1:], joint_pos[-1:]], axis=0))
            next_obs.create_dataset("joint_vel",
                data=np.concatenate([joint_vel[1:], joint_vel[-1:]], axis=0))
            next_obs.create_dataset("rgb",
                data=np.concatenate([rgb[1:], rgb[-1:]], axis=0))
            next_obs.create_dataset("depth",
                data=np.concatenate([depth[1:], depth[-1:]], axis=0))

        data_grp.attrs["total"] = len(npz_files)

        # Train/Valid 분할 (90/10)
        all_demos = [f"demo_{i}" for i in range(len(npz_files))]
        split = int(0.9 * len(all_demos))
        mask = f.create_group("mask")
        mask.create_dataset("train", data=all_demos[:split])
        mask.create_dataset("valid", data=all_demos[split:])

# 실행
npz_to_robomimic_hdf5("~/teaching_data", "indy7_demos.hdf5")
```

#### 학습 워크플로우

```bash
# 1. URDF → USD 변환
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  indy7.urdf indy7.usd --merge-joints --fix-base

# 2. Robomimic으로 모방학습 (BC-RNN)
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
  --task Isaac-Indy7-v0 --algo bc --dataset indy7_demos.hdf5

# 3. PPO로 강화학습 파인튜닝
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Indy7-v0 --num_envs 1024 --headless
```

#### 장단점

| 장점 | 단점 |
|------|------|
| GPU 가속 병렬 환경 (1024+) | NVIDIA GPU 필수 (RTX 3070+) |
| 높은 물리 시뮬레이션 정확도 (PhysX 5) | 설치 용량 ~30 GB |
| Domain Randomization 내장 | 학습 곡선이 가파름 |
| IL → RL 파이프라인 완성도 높음 | NVIDIA 생태계에 종속 |

---

### 7.2 Hugging Face LeRobot

**개요:** 가장 접근성이 좋은 프레임워크. pip 설치만으로 ACT, Diffusion Policy 등 최신 모방학습 알고리즘 사용 가능. 실제 로봇 배포에 초점.

#### 데이터 변환: NPZ → LeRobot 데이터셋

LeRobot은 Parquet (테이블형 데이터) + MP4/PNG (이미지) 구조를 사용합니다:

```
my_indy7_dataset/
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet    # 관절 상태 + 액션
│       └── episode_000001.parquet
├── videos/
│   └── chunk-000/
│       └── observation.images.rgb/
│           ├── episode_000000.mp4    # RGB 영상
│           └── episode_000001.mp4
└── meta/
    ├── info.json                     # 데이터셋 메타정보
    ├── episodes.jsonl
    └── tasks.jsonl
```

**변환 코드:**

```python
import numpy as np
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def npz_to_lerobot(npz_dir, repo_id, fps=15):
    """NPZ 에피소드를 LeRobot 데이터셋으로 변환"""
    import glob
    npz_files = sorted(glob.glob(f"{npz_dir}/episode_*_camera.npz"))

    features = {
        "observation.images.rgb": {
            "dtype": "image", "shape": (224, 224, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32", "shape": (18,),  # 6 pos + 6 vel + 6 effort
            "names": ["state"],
        },
        "action": {
            "dtype": "float32", "shape": (6,),   # target joint positions
            "names": ["actions"],
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=repo_id, robot_type="indy7",
        fps=fps, features=features,
    )

    for npz_file in npz_files:
        data = np.load(npz_file)
        joint_pos = data["joint_positions"]
        joint_vel = data["joint_velocities"]
        joint_eff = data["joint_efforts"]
        rgb = data["rgb_images"]

        N = joint_pos.shape[0]
        state = np.concatenate([joint_pos, joint_vel, joint_eff], axis=1)  # (N, 18)
        actions = np.concatenate([joint_pos[1:], joint_pos[-1:]], axis=0)   # (N, 6)

        for i in range(N):
            dataset.add_frame({
                "observation.images.rgb": Image.fromarray(rgb[i]),
                "observation.state": state[i].astype(np.float32).tolist(),
                "action": actions[i].astype(np.float32).tolist(),
                "task": "pipetting demonstration",
            })
        dataset.save_episode()

    dataset.finalize()
    # dataset.push_to_hub()  # Hugging Face Hub에 업로드

# 실행
npz_to_lerobot("~/teaching_data", "sirlab/indy7_pipetting")
```

#### 지원 알고리즘

| 알고리즘 | 유형 | 특징 |
|----------|------|------|
| **ACT** (Action Chunking Transformer) | IL | 액션 청크 예측, 정밀 조작에 강점 |
| **Diffusion Policy** | IL | 반복 디노이징으로 궤적 생성, SOTA |
| **VQ-BeT** | IL | VQ-VAE + Transformer, 다중 모달 행동 |
| **TDMPC** | RL | 모델 기반 RL, 월드 모델 학습 |
| **Pi0 / SmolVLA** | VLA | 비전-언어-액션, 언어 조건부 조작 |

#### 학습 및 배포

```bash
# 학습 (ACT 정책)
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=sirlab/indy7_pipetting \
  --output_dir=outputs/act_indy7 \
  --batch_size=32 --steps=100000

# 학습 (Diffusion Policy)
lerobot-train \
  --policy.type=diffusion \
  --dataset.repo_id=sirlab/indy7_pipetting \
  --output_dir=outputs/diffusion_indy7
```

**ROS2 배포 노드 (예시):**

```python
class LeRobotInferenceNode(Node):
    """학습된 LeRobot 정책을 ROS2로 실행"""
    def __init__(self):
        super().__init__("lerobot_inference")
        self.policy = torch.load("outputs/act_indy7/pretrained_model")
        self.policy.eval()

        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.rgb_callback, 10)
        self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.timer = self.create_timer(1/15, self.inference_step)

    def inference_step(self):
        observation = {
            "observation.state": torch.tensor(self.current_state),
            "observation.images.rgb": self.current_image,
        }
        with torch.no_grad():
            action = self.policy.select_action(observation)
        cmd = JointState()
        cmd.position = action["action"].cpu().numpy().tolist()
        self.cmd_pub.publish(cmd)
```

#### 장단점

| 장점 | 단점 |
|------|------|
| pip 설치만으로 사용 가능 | 물리 시뮬레이터 미포함 |
| ACT, Diffusion Policy 등 최신 알고리즘 | RL 지원 제한적 |
| Hugging Face Hub으로 데이터셋 공유 | ROS2 통합은 직접 구현 필요 |
| 단일 GPU로 학습 가능 | Depth 이미지 지원 미성숙 |
| 실제 로봇 배포에 초점 | 데이터셋 API 변화 중 (v2→v3) |

---

### 7.3 Unity ML-Agents

**개요:** Unity 엔진의 시각적 렌더링 강점을 활용. GAIL(적대적 모방학습)을 통해 제한된 시연 데이터에서 RL 탐색 학습 가능.

#### 환경 설정

1. **URDF 가져오기**: Unity의 URDF Importer로 `indy7.urdf` → ArticulationBody 변환
2. **ROS2 연동**: ROS-TCP-Connector로 Unity ↔ ROS2 실시간 통신
3. **Agent 스크립트**: `CollectObservations()`, `OnActionReceived()`, `Heuristic()` 구현

```
Unity (C#)  ←─TCP─→  ROS-TCP-Endpoint (ROS2 노드)  ←─DDS─→  ROS2 네트워크
```

#### 데이터 변환: NPZ → .demo

Unity ML-Agents는 Protobuf 기반 `.demo` 파일을 사용합니다. 외부 데이터 변환이 복잡하므로 Unity 내에서 직접 녹화하는 것을 권장합니다.

```python
# 간략한 구조 (실제 구현은 protobuf 직렬화 필요)
# .demo 파일 구조:
#   [32 bytes 헤더]
#   [DemonstrationMetaProto]  - 이름, 스텝 수
#   [BrainParametersProto]    - 관측/액션 스펙
#   [AgentInfoActionPairProto] × N  - 각 스텝의 관측+액션
```

**대안: ROS2 → Unity 실시간 전달**
```bash
# ROS2 TCP 엔드포인트 실행
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

Unity에서 `/joint_states`를 구독하여 실시간으로 시연 데이터를 수신하고, `DemonstrationRecorder`로 .demo 파일 자동 생성.

#### 학습 설정 (GAIL + PPO)

```yaml
# config/indy7_gail.yaml
behaviors:
  Indy7Agent:
    trainer_type: ppo
    hyperparameters:
      batch_size: 1024
      buffer_size: 10240
      learning_rate: 3.0e-4
    network_settings:
      normalize: true
      hidden_units: 256
      num_layers: 3
    reward_signals:
      gail:
        strength: 1.0
        gamma: 0.99
        demo_path: ./demos/indy7.demo
        use_actions: true
      extrinsic:
        strength: 0.0  # 순수 모방 시 0, 하이브리드 시 >0
    max_steps: 500000
```

```bash
mlagents-learn config/indy7_gail.yaml --run-id=indy7_gail_v1
```

#### 장단점

| 장점 | 단점 |
|------|------|
| GAIL로 적대적 모방학습 가능 | 로봇 전용 도구 부족 |
| 높은 시각적 렌더링 품질 | .demo 형식 외부 변환 어려움 |
| ROS2 통신 지원 | GPU 병렬 환경 미지원 |
| TensorBoard 학습 시각화 | 물리 정확도 Isaac Sim 대비 낮음 |
| BC + GAIL + RL 하이브리드 가능 | 로봇 커뮤니티 규모 작음 |

---

### 7.4 프레임워크 비교 요약

| 항목 | Isaac Lab + Robomimic | LeRobot | Unity ML-Agents |
|------|----------------------|---------|-----------------|
| **데이터 형식** | HDF5 | Parquet + MP4 | Protobuf .demo |
| **변환 난이도** | 중간 | 중간 | 높음 |
| **IL 알고리즘** | BC, BC-RNN, BCQ | ACT, Diffusion, VQ-BeT | BC, GAIL |
| **RL 알고리즘** | PPO (RSL-RL, SKRL, SB3) | TDMPC | PPO, SAC |
| **GPU 병렬** | 1024+ 환경 | 학습만 | 단일 환경 |
| **물리 정확도** | 매우 높음 (PhysX 5) | 시뮬레이터 없음 | 중간 |
| **설치 복잡도** | 매우 높음 (~30 GB) | 낮음 (pip) | 중간 (Unity) |
| **실제 로봇 배포** | 간접 | 직접 지원 | ROS-TCP 연동 |

**권장 순서:**

1. **LeRobot으로 시작** → 빠른 프로토타이핑, ACT/Diffusion Policy 즉시 사용
2. **Isaac Lab으로 확장** → 시뮬레이션 데이터 증강, Domain Randomization, RL 파인튜닝
3. **Unity는 선택적** → GAIL이 필요하거나 Unity 기반 환경이 이미 있는 경우

---

## 8. 향후 로드맵

### Phase 1: 직접교시 기반 데이터 수집 ✅ 완료

- [x] Indy7 직접교시 모드 구현
- [x] 20 Hz 관절 데이터 기록
- [x] 에러 복구 및 안전 모니터링
- [x] RealSense D435 카메라 통합
- [x] RGB + Depth 동기화 기록 (~15 Hz)
- [x] 224×224 이미지 리사이즈

### Phase 2: 그리퍼 통합 (Mark 7)

- [ ] 시리얼 통신 ROS2 드라이버 개발 (`mandro_mark7_ros2`)
- [ ] 그리퍼 상태 `/gripper/status` 퍼블리싱 (20 Hz)
- [ ] 키보드 제어 통합: `[G]` 잡기, `[O]` 펴기, `[P]` 파이펫 누르기
- [ ] `gripper_actions` 필드를 동기화 데이터에 추가 (이산 액션: 0/1/2/3)
- [ ] 그리퍼 행동을 `ApproximateTimeSynchronizer`에 통합
- [ ] 파이펫팅 시퀀스 테스트 (잡기 → 이동 → 누르기 → 이동 → 놓기)

### Phase 3: 데이터 변환 파이프라인

- [ ] NPZ → HDF5 (Robomimic) 자동 변환 스크립트
- [ ] NPZ → LeRobot 데이터셋 변환 스크립트
- [ ] 데이터 시각화 및 품질 검증 도구
- [ ] Hugging Face Hub 업로드 자동화

### Phase 4: 모방학습

- [ ] LeRobot ACT / Diffusion Policy 학습
- [ ] 학습된 정책의 ROS2 추론 노드 구현
- [ ] 실제 로봇 배포 테스트
- [ ] 성능 평가 메트릭 정의 (성공률, 궤적 오차)

### Phase 5: Isaac Sim 디지털 트윈

- [ ] Indy7 URDF → USD 변환
- [ ] Isaac Sim 환경 구축
- [ ] 실시간 궤적 미러링 (ROS2 Bridge)
- [ ] Domain Randomization 설정
- [ ] RL 파인튜닝 (PPO, 1024 병렬 환경)

### Phase 6: Sim-to-Real 배포

- [ ] 시뮬레이션 검증 통과 정책 선별
- [ ] 실제 로봇 배포 및 파이펫팅 테스트
- [ ] 피드백 루프: 실패 사례 추가 수집 → 재학습

---

## 참고 자료

- [Neuromeka Indy7 공식 문서](https://neuromeka.com)
- [Intel RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [NVIDIA Isaac Lab](https://github.com/isaac-sim/IsaacLab)
- [Robomimic](https://robomimic.github.io)
- [Hugging Face LeRobot](https://github.com/huggingface/lerobot)
- [Unity ML-Agents](https://github.com/Unity-Technologies/ml-agents)
- [ACT (Action Chunking with Transformers)](https://github.com/tonyzhaozh/act)
- [Diffusion Policy](https://github.com/real-stanford/diffusion_policy)

---

**문서 버전:** v1.0
**최종 수정:** 2026-02-10
**작성:** sirlab
