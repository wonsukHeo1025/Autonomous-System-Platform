# UAV-UGV Cooperation-Based Autonomous System Platform


## 프로젝트 개요
하늘의 눈(UAV)과 땅의 발(UGV)이 만나, 인간이 접근하기 어려운 재난 상황 속에서 정보를 수집하는 ROS2 및 Gazebo 시뮬레이션 환경 기반의 재난 지역 탐사 프로젝트

![image](https://github.com/user-attachments/assets/05c0f056-8759-4bab-bb83-4b3c6ba25cf8)

![image](https://github.com/user-attachments/assets/2a240d2c-8fde-43d9-b4c3-d03a5f92eac3)


## 프로젝트 소개
본 프로젝트는 UAV-UGV 협력 탐사 시스템으로 ROS2와 Gazebo 시뮬레이션 환경을 기반으로 개발되었습니다.

프로젝트의 핵심 시나리오는 다음고 같습니다. 먼저, UGV(무인 지상 차량)가 재난 지역 내 임무 수행 지점까지 UAV(무인 항공기)를 탑재하여 이동합니다. 그 후, UAV는 UGV로부터 이륙하여 UGV가 접근할 수 없는 위험 구역을 정찰하며 주요 정보(ArUco 마커의 ID 및 위치)를 수집합니다. 마지막으로 UAV는 지정된 지점(Rendezvouos Point)으로 이동한 UGV 위에 정밀하게 착륙하여 임무를 완수합니다.

이러한 과정을 통해 자율 시스템의 설계, 다중 로봇 간의 통신, 그리고 시뮬레이션 환경에서의 알고리즘 검증 및 평가 능력을 종합적으로 구현하는 것을 목표로 합니다.


## 주요 기능
- Mission 1: UGV 운송
  UAV를 탑재한 UGV가 드론이 이륙할 수 있는 지점까지 이동합니다. UAV가 이륙할 수 있는 장소에 도착하면 UAV에 이륙 신호를 보냅니다.

- Mission 2: UAV 정찰
  UGV 위에서 UAV가 이륙한 후, 탐사 지점들을 자율 비행하며 ArUco 마커의 ID와 월드 좌표계 기준 위치(X,Y,Z)를 탐지하고 저장합니다. 탐사가 완료되면 Rendezvous Point로 이동합니다.

- Mission 3: UGV 이동 및 장애물 회피
  UGV는 UAV가 임무를 수행하는 동안 Rendezvous Point로 이동합니다. 이 과정에서 2D LiDAR 센서 데이터를 활용하여 경로상의 장애물을 회피하며 이동합니다.

- Mission 4: UAV 정밀 착륙
  UAV는 카메라를 이용해 UGV 상단의 착륙용 마커를 인식하고, 이를 기반으로 정밀 제어하여 UGv 위에 안전하게 착륙합니다.


## 주요 기능 (Key Features)
- ROS2 기반 분산 시스템: 중앙 마스터 없는 노드 간 호율적인 실시간 통신
- LiDAR 기반 장애물 회피: 2D LiDAR 데이터를 이용한 장애물 회피 주행
- ArUco 마커 탐지: 카메라 영상을 이용해 마커의 ID와 3D 위치 정보 추출
- 정밀 착륙 제어: 시각 정보를 이용한 PID 제어로 UGV 상판 위 정밀 착륙
- Gazebo 시뮬레이션: 실제와 유사한 환경에서 전체 미션 검증 및 디버깅


## 사전 준비물 (Prerequisites)
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic


## 좌표계 관리 (TF - Transform)
```
graph TD
    A[world] --> B(ugv_base_link);
    B --> C(uav_base_link);
    C --> D(camera_link);
    D --> E(aruco_marker);
```
world(map) 좌표계를 기준으로 UGV, UAV, 카메라, 그리고 최종적으로 인식된 ArUco 마커까지의 관계가 위와 같은 트리 구조로 연결됩니다.

1. 웨이포인트 추종 (Waypoint Following)
모든 임무의 기준이 되는 웨이포인트는 고정된 world 좌표계에 정의되어 있습니다. 로봇(UAV, UGV)이 성공적으로 웨이포인트를 따라가기 위해서는 world 좌표계상에서 자신의 현재 위치와 방향(Pose)을 알아야합니다. tf2는 로봇의 base_link와 world 좌표계 간의 변환 관계를 지속적으로 제공하여, 로봇이 목표 지점까지 이동할 경로를 계산할 수 있게 합니다.

2. ArUco 마커 위치 특정 (ArUco Markere Localization)
UAV 정찰 임무의 최종 목표는 ArUco 마커의 world 좌표를 알아내는 것입니다. 이 과정은 여러 단계의 TF 변환을 통해 이루어집니다.
1. UAV의 카메라는 자신의 camera_link 좌표계를 기준으로 ArUco 마커의 상대 위치를 인식합니다.
2. tf2는 이미 알고 있는 uav_base_link와 camera_link 사이의 정적인 변환(Static Transform)을 사용합니다.
3. world ← uav_base_link ← camera_link ← aruco_marker, 카메라에 포착된 마커의 상대 위치를 최종적인 전역 위치, 즉 world 좌표로 변환합니다.


## 시작하기 (Getting Started)
1. GitHub 저장소 복제 (Clone the repo)
2. ROS2 워크스페이스 설정 (Setup ROS2 Workspace)
   ```
   cd final_ws
   colcon build
   source install/setup.bash
   ```
3. 실행
   ```
   terminal1: ./final.sh
   terminal2: ros2 launch uav_controller uav_controller
   terminal3: ros2 launch ugv_controller path_follower_node
   ```
