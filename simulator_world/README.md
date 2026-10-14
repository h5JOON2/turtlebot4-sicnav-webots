# Simulator World (Webots)

이 디렉토리는 Webots 시뮬레이터 환경 설정 및 월드 파일을 포함합니다.

## 구성 요소
- **worlds/turtlebot4_crowd_world.wbt**: TurtleBot 4 로봇, 보행자(Human), 정적 장애물, 내비게이션 목표 지점이 배치된 14m x 14m 크기의 3D 시뮬레이션 환경입니다.
- **TurtleBot 4**:
  - 베이스: iRobot Create 3 규격 차동구동 섀시 (휠 간격: 0.233m, 휠 반지름: 0.036m)
  - 센서: 360도 2D RPLidar (`/scan`), IMU (`inertial_unit`), 휠 엔코더 (`left/right_wheel_sensor`)
  - 액추에이터: 좌/우 바퀴 모터 (`left_wheel_motor`, `right_wheel_motor`)
- **Pedestrians (Human 0~3)**:
  - 동적 보행자 에이전트. `crowd_supervisor`에 의해 제어되거나 자연스러운 움직임을 생성합니다.
- **Goal Marker**:
  - 초록색 원형 마커로 목표 지점(기본 (0, 5))을 시각화합니다.

