
#ifndef TRUCK_TRAJECTORY_ESTIMATOR_H_
#define TRUCK_TRAJECTORY_ESTIMATOR_H_


/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
//#include <tf/transform_broadcaster.h>

/* qp solver */
#include <qpOASES.hpp>

/* general header file */
#include <iostream>

using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace truck_trajectory_estimator
{
  class TruckTrajectoryEstimator
  {
  public:
    //TruckTrajectoryEstimator();
    //~TruckTrajectoryEstimator();
    ros::NodeHandle nh_;
    bool truck_odom_update_;
    bool first_truck_odom_flag_;
    bool estimating_truck_odom_flag_;
    nav_msgs::Path *truck_traj_path_;
    nav_msgs::Path *truck_origin_path_;
    visualization_msgs::MarkerArray *truck_origin_markers_;
    visualization_msgs::MarkerArray *truck_traj_markers_;
    visualization_msgs::Marker truck_marker_;

    int polynomial_order_;
    int solver_mode_;
    int estimating_pos_number_;
    int current_pos_number_;
    double lambda_D_;
    VectorXd *truck_pos_x_;
    VectorXd *truck_pos_y_;
    VectorXd *truck_pos_time_;
    double estimating_start_time_;
    double estimating_end_time_;

    std::string truck_odom_sub_topic_name_;

    // Subscriber
    ros::Subscriber sub_truck_odom_;

    // Publisher
    ros::Publisher pub_truck_origin_path_;
    ros::Publisher pub_truck_traj_path_;
    ros::Publisher pub_truck_origin_markers_;
    ros::Publisher pub_truck_traj_markers_;

    //void onInit(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void onInit();
    void markerInit(visualization_msgs::Marker &marker, char color='r');
    void truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg);
    void pathEstimator();
    void pathVisualization();
  };
}
#endif
