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


## 실행 방법
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
   terminal3: ros2 launch ugv_controller path_follower
   ```
