// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef ARUCO_EKF_SLAM_H
#define ARUCO_EKF_SLAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/aruco.hpp>
#include <tf/tf.h>

class Observation
{
public:
    Observation(){}
    Observation(const int& aruco_id, const double& r, const double& phi): aruco_id_(aruco_id), r_(r), phi_(phi){}
    int aruco_id_;
    double r_;
    double phi_;
};// class Observation

class ArUcoEKFSLAM{
public:
    ArUcoEKFSLAM(const cv::Mat& K, const cv::Mat& dist, 
                 const double& kl, const double kr, const double& b, 
                 const Eigen::Matrix4d& T_r_c, 
                 const double& k, const double& k_r, const double k_phi, 
                 const int&  n_markers, const int& marker_size, const double& marker_length);

    void addEncoder(const double& el, const double& er); //加入编码器数据进行运动更新
    void addImage(const cv::Mat& img); // 加入图像数据进行观测更新
    
    visualization_msgs::MarkerArray toRosMarkers(double scale); //将路标点转换成ROS的marker格式，用于发布显示
    geometry_msgs::PoseWithCovarianceStamped toRosPose(); //将机器人位姿转化成ROS的pose格式，用于发布显示
    
    Eigen::MatrixXd& mu(){return mu_;}
    Eigen::MatrixXd& sigma(){return sigma_;}
    cv::Mat markedImg(){return marker_img_;}
    
private:
    int getObservations(const cv::Mat& img, std::vector<Observation>& obs);
    void normAngle(double& angle);
    bool checkLandmark(int aruco_id, int& landmark_idx); 

    /* 系统状态 */
    bool is_init_;
    
    /* 系统配置参数 */
    cv::Mat K_, dist_; //　相机内参数
    double kl_, kr_, b_; // 里程计参数
    Eigen::Matrix4d T_r_c_; // 机器人外参数
    double k_; // 里程计协方差参数
    double k_r_; // 观测协方差参数
    double k_phi_; // 观测协方差参数
    
    int n_markers_;
    int marker_size_;
    double marker_length_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat marker_img_;
    
    /* 上一帧的编码器读数 */
    double last_enl_, last_enr_;
    
    /* 求解的扩展状态 均值 和 协方差 */
    Eigen::MatrixXd mu_; //均值
    Eigen::MatrixXd sigma_; //方差
    std::vector<int> aruco_ids_; //对应于每个路标的aruco码id
    
}; //class ArUcoEKFSLAM

#endif
