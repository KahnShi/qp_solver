#ifndef QUADROTOR_COMMAND_H_
#define QUADROTOR_COMMAND_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
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
    Vector4d m_uav_q;
    // nav_mags::Odometry uav_odom_;
    //nav_mags::Odometry truck_odom_;
    std::string m_truck_odom_sub_topic_name;
    std::string m_uav_odom_sub_topic_name;

    // Subscriber
    ros::Subscriber m_sub_truck_odom;
    ros::Subscriber m_sub_uav_odom;

    // Publisher
    ros::Publisher m_pub_truck_origin_path;

    // Configure
    //boost::shared_ptr<dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig> > m_server_pt_;

    void onInit();
    void truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg);
    void uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_odom_msg);
    bool isUavTruckNear(double threshold);

  };
}

#endif
