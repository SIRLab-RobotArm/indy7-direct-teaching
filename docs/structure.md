# 파이펫 텔레오퍼레이션

**[프로젝트 기획서] 직접 교시 기반 피지컬 AI 데이터 수집 및 학습 시스템 구축
1. 프로젝트 개요**
• **목표**: 실제 로봇(Indy7 + Mark 7)을 직접 움직여 얻은 시연 데이터를 바탕으로, 아이작 심(Isaac Sim) 내 디지털 트윈 환경에서 모방 학습 및 강화 학습을 수행하여 정밀한 파이펫팅(Pipetting) 지능을 구현함.
• **핵심 기술 스택**:
    ◦ **Hardware**: Indy7 (Robot Arm), Mand.ro Mark 7 (Gripper), Intel RealSense D435 (Camera)
    ◦ **Software**: ROS2 (Humble/Foxy), Isaac Sim, Isaac Lab & Robomimic, LeRobot
    ◦ **Algorithm**: Behavior Cloning, Diffusion Policy, PPO (Reinforcement Learning)
**2. 시스템 요구사항 (System Requirements)
2.1 하드웨어 인터페이스 요구사항**
• **로봇 팔(Indy7)**:
    ◦ ROS2 기반 실시간 관절 각도($q$) 및 속도($\dot{q}$) 피드백 수집.
    ◦ 데이터 수집을 위한 **직접 교시(Direct Teaching)** 모드 활성화 및 충돌 감지 해제 기능 필수.
• **그리퍼(Mark 7)**:
    ◦ 115200bps 시리얼 통신을 통한 15-byte 제어 패킷 전송 및 19-byte 상태 패킷 수집.
    ◦ 파이펫팅 버튼 조작을 위한 개별 손가락(특히 엄지) 정밀 위치 제어.
• **카메라(D435)**:
    ◦ RGB 영상(학습 입력용) 및 Aligned Depth 영상(거리 측정용) 동시 수집.
    ◦ 로봇 엔드이펙터에 부착하여 시점(Point of View) 데이터 확보.
**2.2 소프트웨어 및 기능 요구사항**
• **데이터 동기화**: 영상, 관절 각도, 그리퍼 상태 데이터가 동일한 타임스탬프를 공유하도록 `ApproximateTimeSynchronizer` 구현.
• **데이터 변환**: 수집된 `rosbag2` 데이터를 Robomimic 표준 **HDF5** 포맷으로 자동 변환하는 툴체인 구축.
• **디지털 트윈**: 실제 로봇의 움직임을 Isaac Sim 내 가상 로봇이 1:1로 실시간 추종(Mirroring).

3. 확장된 ROS2 워크스페이스 구조

```jsx
pipet_physical_ai_ws/
├── src/
│   ├── hardware_drivers/
│   │   ├── indy7_ros2/           # Indy7 공식 제어 드라이버
│   │   ├── mandro_mark7_ros2/    # [신규] 그리퍼 시리얼 패킷 통신 노드
│   │   └── realsense-ros/        # [신규] D435 카메라 드라이버 패키지
│   ├── perception_processing/
│   │   └── indy7_vision_proc/    # 이미지 리사이징(224x224) 및 전처리 노드
│   ├── teleop_control/
│   │   └── indy7_gripper_teleop/ # 팔(직접교시) + 그리퍼(조종) 통합 제어 노드
│   ├── data_management/
│   │   ├── indy7_data_logger/    # [신규] rosbag2 기록 및 커스텀 로깅 노드
│   │   └── hdf5_converter/       # ROS2 Bag -> Robomimic HDF5 변환 스크립트
│   └── simulation/
│       └── indy7_isaac_bridge/   # Isaac Sim과의 ROS2 통신용 Action Graph 설정
└── docs/                         # 설계 문서 및 프로토콜 매뉴얼 저장
```

**4. 데이터 수집 파이프라인 (Data Pipeline)**
1. **시연 실행**: 사용자가 Indy7을 잡고 움직이며, 단축키/페달로 그리퍼를 조작해 파이펫팅 시연.
2. **Raw 데이터 저장**: `ros2 bag record`를 통해 `/joint_states`, `/gripper/status`, `/camera/color/image_raw` 등을 기록.
3. **HDF5 변환**: 변환 스크립트가 영상을 배열로 변환하고, 시점 $t$의 관측값과 $t+1$의 관절 위치를 Action으로 매핑.
4. **Sim-to-Real 고도화**:
    ◦ **Robomimic**: 변환된 데이터로 초기 모방 학습 모델 생성.
    ◦ **Isaac Lab**: 모방 학습 모델을 초기값으로 하여 강화 학습(RL) 수행.