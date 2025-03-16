# rtabmap_utils v2.0 (2023)
=================================

## *rtabmap_utils* for ROS2 Humble

###### Original Author: David Portugal, Ingeniarius Ltd.
###### ROS2 Migration: [Your Name]
 
************
 
이 패키지는 Kinect v2와 RPLIDAR A2를 사용하여 RTAB-Map과 SLAM Toolbox를 실행하기 위한 ROS2 런치 파일을 포함하고 있습니다.
또한 pushcart mesh, URDF 파일, 그리고 rviz에서 실험 시각화를 위한 rviz 설정 파일도 포함되어 있습니다.

## 설치 방법

```bash
# ROS2 Humble 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 이 패키지 클론
git clone https://github.com/your-username/rtabmap_utils.git

# 의존성 패키지 설치
sudo apt install ros-humble-rtabmap-ros ros-humble-slam-toolbox ros-humble-rplidar-ros

# Azure Kinect를 사용하는 경우 (Kinect v2 대체)
sudo apt install ros-humble-k4a-ros2

# 빌드
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 사용 방법

### 기본 실행 (Kinect와 RPLidar 실행)
```bash
ros2 launch rtabmap_utils run_kinect_and_rplidar.py
```

### RTAB-Map 실행
```bash
ros2 launch rtabmap_utils rtabmap.py
```

### Bag 파일로 RTAB-Map 실행
```bash
ros2 launch rtabmap_utils rtabmap_with_bag.py
```

### Pushcart 모델 표시
```bash
ros2 launch rtabmap_utils display_pushcart.py
```

## 원본 실험 영상

https://www.youtube.com/watch?v=MlrcTyXy5No

## ROS1 데이터셋

원본 ROS1 bag 데이터셋은 다음 링크에서 다운로드할 수 있습니다:
https://goo.gl/c4e8BS

ROS2에서 사용하려면 다음 명령으로 변환해야 합니다:
```bash
ros2 bag convert -i kinect+rplidar.bag -o kinect+rplidar_ros2
```
