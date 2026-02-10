# Indy7 직접교시 텔레오퍼레이션

Indy7 로봇을 위한 직접교시 모드 및 데이터 수집 패키지입니다. 모방학습(Imitation Learning) 및 Physical AI 응용을 위해 설계되었습니다.

## 개요

이 패키지는 Indy7 로봇에서 **직접교시 모드**를 활성화하여, 사용자가 로봇 팔을 손으로 직접 움직이면서 시연 데이터를 자동으로 기록할 수 있게 합니다. 수집된 데이터는 다음 용도로 사용할 수 있습니다:

- 행동 복제 / 모방학습 (Behavior Cloning / Imitation Learning)
- Diffusion Policy 학습
- 시연 기반 강화학습
- Isaac Sim 디지털 트윈 학습

## 기능

- ✅ 중력 보상을 통한 직접교시 모드
- ✅ 20 Hz 실시간 관절 상태 기록
- ✅ 키보드 기반 에피소드 녹화 제어
- ✅ 다중 데이터 형식 지원 (pickle, numpy, JSON 메타데이터)
- ✅ 자동 에피소드 관리
- ✅ 안전 모니터링 (충돌 감지, 관절 한계)
- ✅ **카메라 통합** (RealSense D435) - RGB + Depth 동기화 기록
- 🔄 예정: 그리퍼 통합 (Mark 7)

## 설치

### 사전 요구사항

- ROS2 Humble
- Indy7 로봇 및 `indy_driver` 패키지 설치
- Python 3.8+
- Neuromeka Python SDK: `pip3 install neuromeka`

### 카메라 사전 요구사항 (선택)

RealSense D435 카메라 통합을 위해:

```bash
# RealSense SDK 및 ROS2 패키지 설치
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-camera

# 카메라 의존성 설치
sudo apt install ros-humble-cv-bridge ros-humble-message-filters ros-humble-image-transport

# Python용 OpenCV 설치 (이미지 리사이징용)
pip3 install opencv-python
```

### 패키지 빌드

```bash
cd ~/pipet_physical_ai_ws
colcon build --packages-select indy7_gripper_teleop
source install/setup.bash
```

## 빠른 시작

### Step 0: 네트워크 설정 (최초 1회)

로봇과 컴퓨터를 USB 이더넷 어댑터로 연결한 경우:

```bash
# USB 이더넷 어댑터에 고정 IP 설정
sudo ip addr add 192.168.1.100/24 dev enx00e04c360046

# 로봇 연결 확인
ping 192.168.1.10
```

### Step 1: Terminal 1 - 로봇 드라이버 실행

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

# 직접교시 노드 실행
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

## 카메라 통합

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

### 카메라 토픽 확인

```bash
# 카메라 토픽이 발행되고 있는지 확인
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

### 카메라 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `enable_camera` | `false` | 카메라 녹화 활성화 |
| `enable_depth` | `true` | Depth 이미지 기록 |
| `resize_images` | `true` | 224x224로 리사이즈 (학습용) |
| `camera_fps` | `15` | 카메라 프레임 레이트 |

### 카메라 데이터 형식

카메라 모드 활성화 시 저장되는 데이터:

```python
{
    'timestamps': np.array([...]),           # 동기화된 타임스탬프
    'joint_positions': np.array([...]),      # 관절 각도 (radians)
    'joint_velocities': np.array([...]),     # 관절 속도 (rad/s)
    'joint_efforts': np.array([...]),        # 관절 토크 (N·m)
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

### 카메라 데이터 확인

```python
import numpy as np

# NPZ 파일 로드
data = np.load('~/teaching_data/episode_001_XXXXXX_camera.npz')

print("포함된 키:", list(data.keys()))
print("RGB 이미지:", data['rgb_images'].shape)    # (N, 224, 224, 3)
print("Depth 이미지:", data['depth_images'].shape) # (N, 224, 224)
print("Joint positions:", data['joint_positions'].shape)  # (N, 6)
```

---

## 키보드 조작

| 키 | 동작 |
|----|------|
| `SPACE` | 에피소드 녹화 시작/중지 |
| `H` | 로봇을 홈 위치로 이동 |
| `S` | 로봇 상태 표시 (관절 위치, 한계) |
| `E` | 에러 복구 (빨간 LED 시 사용) |
| `V` | 카메라/동기화 상태 표시 (카메라 활성화 시) |
| `R` | 에피소드 카운터 리셋 |
| `C` | 화면 지우고 안내 표시 |
| `Q` | 종료 및 직접교시 모드 비활성화 |

---

## 데이터 형식

### 파일 구조

각 에피소드는 세 가지 형식으로 저장됩니다:

```
~/teaching_data/
├── episode_001_20260210_153045.pkl           # Python pickle 형식
├── episode_001_20260210_153045.npz           # NumPy 배열
├── episode_001_20260210_153045_metadata.json # JSON 메타데이터
├── episode_001_20260210_153045_camera.pkl    # 카메라 모드 (pickle)
├── episode_001_20260210_153045_camera.npz    # 카메라 모드 (numpy)
└── ...
```

### 데이터 내용

**관절 전용 모드:**
```python
{
    'timestamps': np.array([...]),               # 상대 타임스탬프 (초)
    'joint_positions': np.array([[...], ...]),   # 관절 각도 (radians)
    'joint_velocities': np.array([[...], ...]),  # 관절 속도 (rad/s)
    'joint_efforts': np.array([[...], ...]),     # 관절 토크 (N·m)
    'joint_names': ['joint0', 'joint1', ...]     # 관절 이름
}
```

**카메라 모드:**
```python
{
    'timestamps': np.array([...]),
    'joint_positions': np.array([...]),
    'joint_velocities': np.array([...]),
    'joint_efforts': np.array([...]),
    'rgb_images': np.array([...]),               # (N, 224, 224, 3), uint8
    'depth_images': np.array([...]),             # (N, 224, 224), uint16
}
```

### Python에서 데이터 로드

```python
import pickle
import numpy as np

# Pickle 형식 로드
with open('episode_001_20260210_153045_camera.pkl', 'rb') as f:
    data = pickle.load(f)

# 또는 NumPy 형식 로드
data = np.load('episode_001_20260210_153045_camera.npz')
positions = data['joint_positions']
rgb_images = data['rgb_images']
depth_images = data['depth_images']
```

---

## 아키텍처

### 컴포넌트

1. **TeachingModeManager** ([teaching_mode_manager.py](indy7_gripper_teleop/teaching_mode_manager.py))
   - 로봇 직접교시 모드 활성화/비활성화 관리
   - `indy_srv` 서비스와 인터페이스
   - 복구 및 홈 위치 명령 처리

2. **DataLogger** ([data_logger.py](indy7_gripper_teleop/data_logger.py))
   - `/joint_states` 토픽 구독
   - 녹화 중 20 Hz로 데이터 수집
   - 다중 형식으로 에피소드 저장

3. **CameraDataLogger** ([camera_data_logger.py](indy7_gripper_teleop/camera_data_logger.py))
   - 동기화된 관절 + 카메라 데이터 수집
   - `ApproximateTimeSynchronizer`를 사용한 다중 센서 정렬
   - ~15 Hz로 RGB 및 Depth 이미지 기록
   - 모방학습을 위해 224x224로 이미지 리사이즈

4. **DirectTeachingNode** ([direct_teaching_node.py](indy7_gripper_teleop/direct_teaching_node.py))
   - 메인 조정 노드
   - 키보드 입력 처리
   - 상태 퍼블리싱

### 토픽

**구독 (관절 전용 모드):**
- `/joint_states` (sensor_msgs/JointState) - 로봇 관절 피드백

**구독 (카메라 모드):**
- `/joint_states` (sensor_msgs/JointState) - 로봇 관절 피드백
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - RGB 이미지
- `/camera/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image) - 정렬된 Depth

> **참고:** 카메라 토픽은 이중 네임스페이스 (`/camera/camera/...`)를 사용합니다. RealSense 노드가 네임스페이스 `camera`와 노드 이름 `camera`로 실행되기 때문입니다.

**퍼블리시:**
- `/teaching_status` (std_msgs/String) - 현재 녹화 상태

**서비스:**
- `/indy_srv` (indy_interfaces/IndyService) - 로봇 모드 제어

---

## 문제 해결

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

### 카메라 모드: 0 샘플 기록됨

**증상:** 녹화 결과가 "0 sync frames, 0.0 Hz"

**원인 및 해결:**

1. **잘못된 실행 순서:** 카메라가 직접교시 노드보다 먼저 실행되어야 함
   ```bash
   # 올바른 순서:
   # 1. 로봇 드라이버 → 2. 카메라 → 3. 직접교시 노드
   ```

2. **잘못된 토픽 이름:** 실제 카메라 토픽 이름 확인
   ```bash
   ros2 topic list | grep camera
   # /camera/camera/color/image_raw (이중 네임스페이스) 확인
   ```

3. **카메라가 발행하지 않음:** 카메라 Hz 확인
   ```bash
   ros2 topic hz /camera/camera/color/image_raw
   # ~15-30 Hz 표시되어야 함
   ```

### 카메라 USB 2.0 경고

**경고:** "Device USB type: 2.1 - Reduced performance is expected"

**해결:** 카메라를 USB 허브가 아닌 USB 3.0 포트 (파란색 포트)에 직접 연결하세요.

### 카메라 데이터 기록 확인

```python
import numpy as np
data = np.load('~/teaching_data/episode_XXX_camera.npz')
print("키:", list(data.keys()))
print("RGB 이미지:", data['rgb_images'].shape)   # (N, 224, 224, 3)
print("Depth 이미지:", data['depth_images'].shape) # (N, 224, 224)
```

### 낮은 샘플링 레이트

**예상:** 20 Hz (joint_states 발행 레이트와 동일)

**더 낮은 경우:**
- 시스템 부하 확인: `htop`
- joint_states 주파수 확인: `ros2 topic hz /joint_states`
- 저장 빈도를 줄여서 파일 I/O 감소

---

## 향후 확장

### Phase 2: 그리퍼 통합 (Mark 7)

추가 예정:
- 그리퍼 상태 구독
- 동기화된 그리퍼 위치 기록
- 시연 중 손가락 제어

### ~~Phase 3: 카메라 통합 (RealSense D435)~~ ✅ 완료

구현된 기능:
- ✅ RGB 이미지 캡처 (학습용 224x224)
- ✅ Depth 이미지 캡처 (컬러에 정렬)
- ✅ `ApproximateTimeSynchronizer`를 사용한 다중 센서 동기화
- ✅ 동기화된 타임스탬프로 비전 기반 학습 지원

### Phase 4: Isaac Sim 디지털 트윈

추가 예정:
- Isaac Sim으로 실시간 궤적 미러링
- 양방향 통신
- Sim-to-real 검증

---

## 관련 문서

- 메인 워크스페이스: [CLAUDE.md](../../../CLAUDE.md)
- 프로젝트 구조: [docs/structure.md](../../../docs/structure.md)
- Indy 드라이버: [src/hardware_drivers/indy-ros2/README.md](../../hardware_drivers/indy-ros2/README.md)

## 라이선스

BSD-3-Clause

## 관리자

sirlab

---

**참고:** 이것은 Indy7 팔에만 초점을 맞춘 초기 구현입니다. 그리퍼 통합은 향후 업데이트에서 추가될 예정입니다.
