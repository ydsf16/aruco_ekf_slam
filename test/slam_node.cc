// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <aruco_ekf_slam/aruco_ekf_slam.h>

ArUcoEKFSLAM* g_slam;
void ImageCallback(const sensor_msgs::ImageConstPtr& img_ptr );
void EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr& en_ptr);

ros::Publisher g_landmark_pub;
ros::Publisher g_robot_pose_pub;
image_transport::Publisher g_img_pub;

int main(int argc, char **argv)
{
    /***** 初始化ROS *****/
    ros::init(argc, argv, "reslam_odom"); 
    ros::NodeHandle nh;
    
    /***** 获取参数 *****/
    /* TODO 错误处理 */
    std::string image_topic_name, encoder_topic_name;
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    double kl, kr, b;
    Eigen::Matrix4d T_r_c;
    double k, k_r, k_phi;
    int n_markers, marker_size;
    double marker_length;
        
    nh.getParam ( "/slam_node/topic/image", image_topic_name);
    nh.getParam ( "/slam_node/topic/encoder", encoder_topic_name);
    
    nh.getParam ( "/slam_node/camera/fx", fx);
    nh.getParam ( "/slam_node/camera/fy", fy);
    nh.getParam ( "/slam_node/camera/cx", cx);
    nh.getParam ( "/slam_node/camera/cy", cy);
    nh.getParam ( "/slam_node/camera/k1", k1);
    nh.getParam ( "/slam_node/camera/k2", k2);
    nh.getParam ( "/slam_node/camera/p1", p1);
    nh.getParam ( "/slam_node/camera/p2", p2 );
    nh.getParam ( "/slam_node/camera/k3", k3 );    
    cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat dist = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
    
    nh.getParam ( "/slam_node/odom/kl", kl );
    nh.getParam ( "/slam_node/odom/kr", kr );
    nh.getParam ( "/slam_node/odom/b", b );
 
    std::vector<double> Trc;
    nh.getParam("/slam_node/extrinsic/Trc", Trc);
    T_r_c << Trc[0], Trc[1], Trc[2], Trc[3],
    Trc[4], Trc[5], Trc[6], Trc[7],
    Trc[8], Trc[9], Trc[10], Trc[11],
    0.0, 0.0, 0.0, 1.0;
    
    nh.getParam ( "/slam_node/covariance/k", k );
    nh.getParam ( "/slam_node/covariance/k_r", k_r );
    nh.getParam ( "/slam_node/covariance/k_phi", k_phi );
    
    nh.getParam ( "/slam_node/aruco/n_markers", n_markers);
    nh.getParam ( "/slam_node/aruco/marker_size", marker_size);
    nh.getParam ( "/slam_node/aruco/marker_length", marker_length);

    /***** 初始化 Aruco EKF SLAM *****/
    g_slam = new ArUcoEKFSLAM(K, dist, kl, kr, b, T_r_c, k, k_r, k_phi, n_markers, marker_size, marker_length);
    
    /***** 初始化消息发布 *****/
    g_landmark_pub = nh.advertise<visualization_msgs::MarkerArray>( "ekf_slam/landmark", 1);
    g_robot_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_slam/pose", 1);
    image_transport::ImageTransport it(nh);
    g_img_pub = it.advertise("ekf_slam/image", 1);
        
    /***** 初始化消息订阅 *****/
    ros::Subscriber image_sub = nh.subscribe(image_topic_name, 1, ImageCallback);
    ros::Subscriber encoder_sub = nh.subscribe(encoder_topic_name, 1, EncoderCallback);
    
    std::cout << "\n\nSYSTEM START \n\n";
    ros::spin();
    
    delete g_slam;
    return 0;
}

void ImageCallback ( const sensor_msgs::ImageConstPtr& img_ptr )
{
    cv_bridge::CvImageConstPtr cv_ptr  = cv_bridge::toCvShare ( img_ptr );
    
    /* add image */
    g_slam->addImage(cv_ptr->image);
   
    /* publish markerd image */
    cv::Mat img= g_slam->markedImg();
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
   g_img_pub.publish(msg);
}


void EncoderCallback ( const geometry_msgs::QuaternionStamped::ConstPtr& en_ptr )
{
    /* 四个轮子的编码器整合成两个轮子的编码器数据 */
    double enl1 = en_ptr->quaternion.x;
    double enl2 = en_ptr->quaternion.y;
    double enr1 = en_ptr->quaternion.z;
    double enr2 = en_ptr->quaternion.w;
    double enl = 0.5*(enl1 + enl2);
    double enr = 0.5*(enr1 + enr2);

    /* 加入ＥＫＦ-SLAM */
    g_slam->addEncoder(enl, enr);
    
    /* publish  landmarks */
    visualization_msgs::MarkerArray markers = g_slam->toRosMarkers(3.5); // 为了方便显示协方差做了放大
    g_landmark_pub.publish(markers);
    
    /* publish  robot pose */
    geometry_msgs::PoseWithCovarianceStamped pose = g_slam->toRosPose(); // pose的协方差在rviz也做了放大
    g_robot_pose_pub.publish(pose);
}
