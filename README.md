# 招人啦！！！【长期有效，欢迎交流】【2023-12更新】

嗨，大家好！

我是蔚来汽车[NIO]自动驾驶团队的一员，负责多传感器融合定位、SLAM等领域的研发工作。目前，我们正在寻找新的队友加入我们。

我们团队研发的技术已经在多个功能场景成功量产。例如，**高速城快领航辅助驾驶[NOP+]** 功能于2022年发布，截至2023年10月，已在累积服务里程超过1亿公里。今年，还推出了技术更为复杂的**城区领航**功能，并通过群体智能不断拓展其可用范围。同时，在11月份发布了独特的**高速服务区领航[PSP]** 体验，实现了高速到服务区换电场景的全流程自动化和全程领航体验。此外，我们团队也参与了一些基础功能背后的研发，如AEB、LCC等。未来还有更多令人期待的功能发发布，敬请期待。

如果你对计算机视觉、深度学习、SLAM、多传感器融合、组合惯导等技术有着扎实的背景，不论是全职还是实习，我们都欢迎你加入我们的团队。有兴趣的话，可以通过微信联系我们：**YDSF16**。

[全域领航辅助｜超3倍完成年度目标，提速规划节奏](https://app.nio.com/app/community_content_h5/module_10050/content?id=531584&type=article&is_nav_show=false&wv=lg)

<a href="https://youtu.be/3A5wpWgrHTI" target="_blank"><img src="https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/20231223-004022.jpeg" 
alt="YDS" width="300" height="300"/></a>

NIO社招内推码: B89PQMZ 
投递链接: https://nio.jobs.feishu.cn/referral/m/position/detail/?token=MTsxNzAzMjY0NzE2NTYyOzY5ODI0NTE1OTI5OTgxOTI2NDg7NzI2MDc4NjA0ODI2Mjk2NTU0MQ


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

