#ifndef QUADROTOR_COMMAND_H_
#define QUADROTOR_COMMAND_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* config */
#include <dynamic_reconfigure/server.h>

/* qp solver */
#include <qpOASES.hpp>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

/* general header file */
#include <iostream>

using namespace Eigen;
//USING_NAMESPACE_QPOASES

namespace quadrotor_command
{
  class QuadrotorCommand
  {
  public:
    ros::NodeHandle m_nh;
    double m_uav_vel_ub;
    double m_uav_vel_lb;
    double m_uav_acc_ub;
    double m_uav_acc_lb;
    Vector3d m_uav_world_pos;
    Vector3d m_truck_world_pos;
    Quaterniond m_uav_q;
    int m_control_freq;
    int m_takeoff_flag; //0, not takeoff; 1, taking off; 2, took off
    double m_uav_initial_height;

    // pid
    double m_p_gain;
    double m_i_gain;
    Vector3d m_i_term_accumulation;
    double m_p_term_max;
    double m_i_term_max;


    // nav_mags::Odometry uav_odom_;
    //nav_mags::Odometry truck_odom_;
    std::string m_truck_odom_sub_topic_name;
    std::string m_uav_odom_sub_topic_name;
    std::string m_uav_cmd_pub_topic_name;

    // Subscriber
    ros::Subscriber m_sub_truck_odom;
    ros::Subscriber m_sub_uav_odom;

    // Publisher
    ros::Publisher m_pub_truck_origin_path;
    ros::Publisher m_pub_uav_cmd;

    // Configure
    //boost::shared_ptr<dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig> > m_server_pt_;

    void onInit();
    void truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg);
    void uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_odom_msg);
    bool isUavTruckNear(double threshold);
    void pidTracking();
  };
}

#endif
