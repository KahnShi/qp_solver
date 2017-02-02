
#ifndef TRUCK_TRAJECTORY_ESTIMATOR_H_
#define TRUCK_TRAJECTORY_ESTIMATOR_H_


/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
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
    ros::NodeHandle m_nh;
    bool m_truck_odom_update;
    bool m_truck_odom_empty_flag;
    bool m_truck_odom_filled_flag;
    nav_msgs::Path *m_truck_traj_path_ptr;
    nav_msgs::Path *m_truck_origin_path_ptr;
    nav_msgs::Path *m_uav_des_traj_path_ptr;
    visualization_msgs::MarkerArray *m_truck_origin_markers_ptr;
    visualization_msgs::MarkerArray *m_truck_traj_markers_ptr;
    visualization_msgs::Marker m_truck_marker;

    int m_truck_traj_order;
    int m_truck_traj_dev_order;
    int m_n_truck_estimate_odom;
    int m_n_current_odom;
    int m_truck_traj_generate_freq;
    double m_truck_vis_predict_time;
    double m_truck_vis_predict_time_unit;
    double m_truck_vis_preview_time;
    int m_n_truck_new_odom;
    double m_truck_lambda_D;
    double m_truck_smooth_forward_time;
    VectorXd *m_truck_pos_x_ptr;
    VectorXd *m_truck_pos_y_ptr;
    VectorXd *m_truck_vel_x_ptr;
    VectorXd *m_truck_vel_y_ptr;
    VectorXd *m_truck_odom_time_ptr;
    VectorXd *m_truck_traj_param_x_ptr;
    VectorXd *m_truck_traj_param_y_ptr;
    double m_truck_estimate_start_time;
    double m_truck_traj_start_time;
    double m_estimate_end_time;
    double m_truck_traj_deviation_threshold; // If truck largely deviate trajectory, then recalculate its polynomial function
    double m_truck_max_vel; // max velocity in one demension
    double m_truck_max_acc; // max acceleration in one demension
    nav_msgs::Odometry m_truck_odom;
    double m_truck_cable_height; // height of truck's back cable, in gazebo it is 0.5 meter

    std::string m_truck_odom_sub_topic_name;

    /* uav command */
    bool m_is_uav_traj_generate;
    QuadrotorCommand m_uav_commander;
    int m_uav_state; //0,not finish taking off; 1,pid tracking; 2,traj trakcing
    int m_uav_traj_order;
    int m_uav_traj_dev_order;
    double m_uav_lambda_D;
    VectorXd *m_uav_traj_param_x_ptr;
    VectorXd *m_uav_traj_param_y_ptr;
    VectorXd *m_uav_prev_traj_param_x_ptr;
    VectorXd *m_uav_prev_traj_param_y_ptr;
    double m_uav_prev_traj_start_time;
    double m_uav_prev_traj_start_height;
    bool m_uav_traj_planning_flag;
    double m_uav_landing_time;
    double m_uav_landing_vel;
    // count for times when uav is state 2
    int m_uav_state2_cnt;
    double m_uav_landing_sample_gap;

    // Subscriber
    ros::Subscriber m_sub_truck_odom;

    // Publisher
    ros::Publisher m_pub_truck_origin_path;
    ros::Publisher m_pub_truck_traj_path;
    ros::Publisher m_pub_uav_des_traj_path;
    ros::Publisher m_pub_truck_origin_markers;
    ros::Publisher m_pub_truck_traj_markers;

    // Configure
    boost::shared_ptr<dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig> > m_server_ptr;
    void traj_estimate_config_callback(quadrotor_trajectory::TrajectoryEstimateConfig &config, uint32_t level);

    //void onInit(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void onInit();
    void markerInit(visualization_msgs::Marker &marker, char color='r');
    void truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg);
    void pathEstimator();
    void pathVisualization();
    void truckTrajectoryEstimation();
    bool uavTrajectoryPlanning();
    void trajectoryVisualization();
    // trajectory visualization based on same odom points
    void trajectory_visualization_same_odompoints(int mode);
    int factorial(int n, int order);
    double getPointFromTruckTrajectory(char axis, double var_value);
    Vector3d nOrderTruckTrajectory(int n, double t);
    /* truck moves in constant speed, but generated polynomial trajectory is not wonderful constant
       eg. if we want 2s later's position or velocity of truck, if we directly use add 2s in truck trajectory function,
       we may get wrong result. Current solution is assume in future period, truck's speed is constant (eg. 4m/s).
       so 2s later means truck moves 8m forward. so look forward by seperate trajectory into several segments to get 8m's location.
       Finally return to the related "time" with respect to trajectory function.
     */
    double realTimeCvtToTruckTrajectoryTime(double t, Vector3d truck_vel);
    Vector3d nOrderUavTrajectory(int n, double t);
    bool isTruckDeviateTrajectory(double threshold, geometry_msgs::Point truck_pos, double current_time);
  };
}
#endif
