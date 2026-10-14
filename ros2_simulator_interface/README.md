# ROS 2 <-> Simulator Interface

이 디렉토리는 Webots 시뮬레이터와 ROS 2 간의 인터페이스 노드 및 컨트롤러를 포함합니다.

## 구성 요소
- **controllers/turtlebot4_controller/**:
  - `turtlebot4_controller.py`: Webots의 Robot API를 사용하여 TurtleBot 4의 모터를 제어하고, 센서 데이터를 읽어 ROS 2 토픽으로 발행합니다.
  - 구독 토픽:
    - `/cmd_vel` (`geometry_msgs/msg/Twist`): 전진 선속도 및 회전 각속도
  - 발행 토픽:
    - `/odom` (`nav_msgs/msg/Odometry`): 로봇 엔코더/IMU 기반 오도메트리
    - `/scan` (`sensor_msgs/msg/LaserScan`): RPLidar 360도 레이저 스캔 데이터
    - `tf`: `odom` -> `base_link` 좌표계 변환
- **controllers/crowd_supervisor/**:
  - `crowd_supervisor.py`: Webots의 Supervisor API를 사용하여 월드 내 보행자들의 이동을 관리하고 상태를 퍼블리시합니다.
  - 발행 토픽:
    - `/crowd/human_states` (`std_msgs/msg/String`): 보행자들의 2D 위치, 속도, 반경이 포함된 JSON 문자열 (SICNav 인터페이스와 직접 연동)
    - `/crowd/human_poses` (`geometry_msgs/msg/PoseArray`): RViz 시각화용 보행자 포즈 배열
- **launch/**:
  - `webots_turtlebot4_sicnav.launch.py`: 전체 ROS 2 인터페이스와 SICNav 브릿지 노드를 일괄 실행하는 런치 파일

