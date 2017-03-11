
#ifndef VEHICLE_TRAJECTORY_ESTIMATOR_H_
#define VEHICLE_TRAJECTORY_ESTIMATOR_H_


/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayDimension.h>
#include <quadrotor_trajectory/TrackParamStamped.h>

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

using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace vehicle_trajectory_estimator
{
  class VehicleTrajectoryEstimator
  {
  public:
    //VehicleTrajectoryEstimator();
    //~VehicleTrajectoryEstimator();
    ros::NodeHandle m_nh;
    bool m_vehicle_odom_update;
    bool m_vehicle_odom_empty_flag;
    bool m_vehicle_odom_filled_flag;
    nav_msgs::Path *m_vehicle_traj_path_ptr;

    int m_vehicle_traj_order;
    int m_vehicle_traj_dev_order;
    int m_n_vehicle_estimate_odom;
    int m_n_current_odom;
    int m_vehicle_traj_generate_freq;
    double m_vehicle_vis_predict_time;
    double m_vehicle_vis_predict_time_unit;
    double m_vehicle_vis_preview_time;
    int m_n_vehicle_new_odom;
    double m_vehicle_lambda_D;
    double m_vehicle_smooth_forward_time;
    VectorXd *m_vehicle_pos_x_ptr;
    VectorXd *m_vehicle_pos_y_ptr;
    VectorXd *m_vehicle_odom_time_ptr;
    VectorXd *m_vehicle_traj_param_x_ptr;
    VectorXd *m_vehicle_traj_param_y_ptr;
    double m_vehicle_estimate_start_time;
    double m_vehicle_traj_start_time;
    double m_estimate_end_time;
    double m_vehicle_traj_deviation_threshold; // If vehicle largely deviate trajectory, then recalculate its polynomial function
    double m_vehicle_max_vel; // max velocity in one demension
    double m_vehicle_max_acc; // max acceleration in one demension
    nav_msgs::Odometry m_vehicle_odom;
    double m_vehicle_cable_height; // height of vehicle's back cable, in gazebo it is 0.5 meter
    bool m_display_param;

    std::string m_vehicle_odom_sub_topic_name;
    std::string m_vehicle_traj_param_pub_topic_name;
    std::string m_vehicle_traj_path_pub_topic_name;

    // Subscriber
    ros::Subscriber m_sub_vehicle_odom;

    // Publisher
    ros::Publisher m_pub_vehicle_traj_param;
    ros::Publisher m_pub_vehicle_origin_path;
    ros::Publisher m_pub_vehicle_traj_path;
    ros::Publisher m_pub_vehicle_origin_markers;
    ros::Publisher m_pub_vehicle_traj_markers;

    void onInit();
    void vehicleOdomCallback(const nav_msgs::OdometryConstPtr& vehicle_odom_msg);
    void vehicleTrajectoryEstimation();
    void trajectoryVisualization();
    // trajectory visualization based on same odom points
    void trajectory_visualization_same_odompoints(int mode);
    int factorial(int n, int order);
    double getPointFromVehicleTrajectory(char axis, double var_value);
    Vector3d nOrderVehicleTrajectory(int n, double t);
    /* vehicle moves in constant speed, but generated polynomial trajectory is not wonderful constant
       eg. if we want 2s later's position or velocity of vehicle, if we directly use add 2s in vehicle trajectory function,
       we may get wrong result. Current solution is assume in future period, vehicle's speed is constant (eg. 4m/s).
       so 2s later means vehicle moves 8m forward. so look forward by seperate trajectory into several segments to get 8m's location.
       Finally return to the related "time" with respect to trajectory function.
     */
    double realTimeCvtToVehicleTrajectoryTime(double t, Vector3d vehicle_vel);
    bool isVehicleDeviateTrajectory(double threshold, geometry_msgs::Point vehicle_pos, double current_time);
  };
}
#endif
