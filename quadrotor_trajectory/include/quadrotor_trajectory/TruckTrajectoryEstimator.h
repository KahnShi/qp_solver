
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
    int m_n_truck_new_odom;
    double m_truck_lambda_D;
    double m_truck_smooth_forward_time;
    VectorXd *m_truck_odom_x_ptr;
    VectorXd *m_truck_odom_y_ptr;
    VectorXd *m_truck_odom_time_ptr;
    VectorXd *m_truck_traj_param_x_ptr;
    VectorXd *m_truck_traj_param_y_ptr;
    double m_truck_estimate_start_time;
    double m_truck_traj_start_time;
    double m_estimate_end_time;
    VectorXd *m_uav_traj_param_x_ptr;
    VectorXd *m_uav_traj_param_y_ptr;

    std::string m_truck_odom_sub_topic_name;

    /* uav command */
    QuadrotorCommand m_uav_commander;
    int m_uav_state; //0,not finish taking off; 1,pid tracking; 2,traj trakcing
    tf::Vector3 m_uav_start_pos;

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
    void polynomialEstimation();
    void trajectoryVisualization();
    // trajectory visualization based on same odom points
    void trajectory_visualization_same_odompoints(int mode);
    int factorial(int n, int order);
    double getPointFromPolynomial(char axis, double var_value);
    Vector3d nOrderPolynomial(int n, double t);
  };
}
#endif
