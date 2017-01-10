
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

/* config */
#include <dynamic_reconfigure/server.h>
#include <quadrotor_trajectory/TrajectoryEstimateConfig.h>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>

/* qp solver */
#include <qpOASES.hpp>

/* general header file */
#include <iostream>

/* local library */
#include <quadrotor_trajectory/QuadrotorCommand.h>

using namespace Eigen;
USING_NAMESPACE_QPOASES
using namespace quadrotor_command;

namespace truck_trajectory_estimator
{
  class TruckTrajectoryEstimator
  {
  public:
    //TruckTrajectoryEstimator();
    //~TruckTrajectoryEstimator();
    ros::NodeHandle nh_;
    bool truck_odom_update_;
    bool truck_odom_empty_flag_;
    bool truck_odom_filled_flag_;
    nav_msgs::Path *truck_traj_path_;
    nav_msgs::Path *truck_origin_path_;
    nav_msgs::Path *uav_des_traj_path_;
    visualization_msgs::MarkerArray *truck_origin_markers_;
    visualization_msgs::MarkerArray *truck_traj_markers_;
    visualization_msgs::Marker truck_marker_;

    int polynomial_order_;
    int derivation_order_;
    int estimating_odom_number_;
    int current_odom_number_;
    int traj_generate_freq_;
    double visualization_predict_time_;
    double visualization_predict_time_unit_;
    int new_traj_points_number_;
    double lambda_D_;
    double smooth_forward_time_;
    VectorXd *truck_odom_x_;
    VectorXd *truck_odom_y_;
    VectorXd *truck_odom_time_;
    VectorXd *truck_traj_param_x_;
    VectorXd *truck_traj_param_y_;
    double estimating_start_time_;
    double trajectory_start_time_;
    double estimating_end_time_;

    std::string truck_odom_sub_topic_name_;

    /* uav command */
    QuadrotorCommand uav_commander_;
    int uav_state_; //0,not finish taking off; 1,pid tracking; 2,traj trakcing
    tf::Vector3 uav_start_pos_;

    // Subscriber
    ros::Subscriber sub_truck_odom_;

    // Publisher
    ros::Publisher pub_truck_origin_path_;
    ros::Publisher pub_truck_traj_path_;
    ros::Publisher pub_uav_des_traj_path_;
    ros::Publisher pub_truck_origin_markers_;
    ros::Publisher pub_truck_traj_markers_;

    // Configure
    boost::shared_ptr<dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig> > server_ptr_;
    void traj_estimate_config_callback(quadrotor_trajectory::TrajectoryEstimateConfig &config, uint32_t level);

    //void onInit(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void onInit();
    void markerInit(visualization_msgs::Marker &marker, char color='r');
    void truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg);
    // Estimate truck's path using QP
    void polynomialEstimation();
    // Visualize truck and drone's trajectory
    void trajectoryVisualization();
    int factorial(int n, int order);
    double getPointFromPolynomial(char axis, double var_value);
    Vector3d nOrderPolynomial(int n, double t);
  };
}
#endif
