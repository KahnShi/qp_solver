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
#include <tf/transform_broadcaster.h>

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
    double m_uav_vel_z;
    double m_uav_acc_ub;
    double m_uav_acc_lb;
    tf::Vector3 m_uav_world_pos;
    tf::Vector3 m_uav_world_vel;
    tf::Vector3 m_uav_world_acc;
    tf::Vector3 m_uav_world_ang_vel;
    tf::Vector3 m_truck_world_pos;
    tf::Vector3 m_uav_truck_world_pos;
    tf::Quaternion m_uav_q;
    int m_control_freq;
    int m_takeoff_flag; //0, not takeoff; 1, taking off; 2, took off
    double m_uav_initial_height;
    nav_msgs::Odometry m_uav_odom;
    geometry_msgs::Twist m_uav_cmd;
    geometry_msgs::Twist m_uav_pid_cmd;

    bool m_landing_mode;

    // pid
    bool m_direct_pid_mode;
    double m_direct_p_gain;
    double m_direct_i_gain;
    tf::Vector3 m_direct_i_term_accumulation;
    double m_direct_p_term_max;
    double m_direct_i_term_max;

    double m_traj_track_p_gain;
    double m_traj_track_i_gain;
    double m_traj_track_d_gain;
    tf::Vector3 m_traj_track_i_term_accumulation;
    double m_traj_track_p_term_max;
    double m_traj_track_i_term_max;
    double m_traj_track_d_term_max;

    double m_uav_yaw_i_term_accumulation;

    //test
    int m_move_state;
    bool m_up_down_test;

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
    void updateUavTruckRelPos();
    void directPidTracking(int mode); // 1, directly publish cmd msg; 0, calculate but not publish
    void trajectoryTracking_old(Vector3d uav_des_pos, Vector3d uav_des_vel);
    void trajectoryTracking(Vector3d uav_des_pos, Vector3d uav_des_vel);
  };
}

#endif
