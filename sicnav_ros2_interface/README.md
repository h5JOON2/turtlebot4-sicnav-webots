# SICNav <-> ROS 2 Interface

이 디렉토리는 SICNav(Safe and Interactive Crowd Navigation) 알고리즘과 ROS 2 간의 인터페이스 및 플래너 브릿지 노드를 포함합니다.

## 구성 요소
- **sicnav_ros2_bridge/policy_wrapper.py**:
  - SICNav의 핵심 충돌 회피 알고리즘(CAMPC: Collision Avoidance MPC, DWA, ORCA 등)을 로봇의 상태 및 주변 보행자 관측 데이터와 매핑하는 래퍼 모듈입니다.
  - 최적화 솔버(Acados/CasADi) 설치 여부와 무관하게 부드럽게 동작할 수 있는 Social DWA 폴백 메커니즘을 내장하고 있습니다.
- **sicnav_ros2_bridge/sicnav_ros2_node.py**:
  - ROS 2 노드 (`sicnav_planner_bridge`).
  - 구독 토픽:
    - `/odom` (`nav_msgs/msg/Odometry`): 로봇 현재 위치 및 선속도/각속도
    - `/crowd/human_states` (`std_msgs/msg/String`): 보행자들의 위치 및 속도 (Webots Supervisor 또는 감지 파이프라인에서 수신)
    - `/goal_pose` (`geometry_msgs/msg/PoseStamped`): 내비게이션 목표 지점 (기본값: (0.0, 5.0))
    - `/scan` (`sensor_msgs/msg/LaserScan`): Lidar 센서 데이터
  - 발행 토픽:
    - `/cmd_vel` (`geometry_msgs/msg/Twist`): 계산된 최적 선속도 $v$ 및 각속도 $\omega$를 TurtleBot 4 로봇 모터로 전달
    - `/sicnav/status` (`std_msgs/msg/String`): 목표 도달 거리, 현재 속도, 추적 보행자 수 등 상태 모니터링
- **config/**:
  - `bridge_params.yaml`: 제어 주기, 목표점, 속도 한계치 등 설정

