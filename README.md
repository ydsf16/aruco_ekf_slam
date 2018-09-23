
以Aruco码为路标点，实现EKF-SLAM算法。《概率机器人》第10章。

算法说明请参考：https://zhuanlan.zhihu.com/p/45207081

数据集下载地址：https://pan.baidu.com/s/1EX9CYmdEUR2BJh7v5dTNfA

依赖库：Opencv 3.1, Eigen, ROS

使用方法：

STEP1. 下载到自己的 ROS 工作空间：catkin_ws/src

STEP2. 编译：catkin_make

STEP3. 运行 launch: roslaunch aruco_ekf_slam slam.launch

STEP4. 播放 rosbag: rosbag play aruco_slam_data_qhd1.bag -r 5

