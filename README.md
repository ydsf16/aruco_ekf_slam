
# ArUco-based EKF-SLAM. 

![image](https://github.com/ydsf16/aruco_ekf_slam/blob/master/ekf.gif)

For details, please refer to：https://zhuanlan.zhihu.com/p/45207081

Dataset：https://pan.baidu.com/s/1EX9CYmdEUR2BJh7v5dTNfA

## Prerequisites
Opencv 3.1, Eigen, ROS

## Demo

STEP 1. Download the repository to your ROS workspace：catkin_ws/src

STEP 2. Make：catkin_make

STEP 3. Run launch: roslaunch aruco_ekf_slam slam.launch

STEP 4. Play a rosbag: rosbag play aruco_slam_data_qhd1.bag -r 5

