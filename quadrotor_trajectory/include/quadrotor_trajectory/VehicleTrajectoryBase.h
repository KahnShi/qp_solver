
#ifndef VEHICLE_TRAJECTORY_BASE_H_
#define VEHICLE_TRAJECTORY_BASE_H_


/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayDimension.h>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>

/* qp solver */
//#include <qpOASES.hpp>

/* general header file */
#include <iostream>

using namespace Eigen;
//USING_NAMESPACE_QPOASES

namespace vehicle_trajectory_base
{
  class VehicleTrajectoryBase
  {
  public:
    ros::NodeHandle m_nh;

    int m_vehicle_traj_order;
    VectorXd *m_vehicle_traj_param_x_ptr;
    VectorXd *m_vehicle_traj_param_y_ptr;
    double m_vehicle_traj_start_time;

    void onInit(int order, std::vector<double> &data);
    int factorial(int n, int order);
    Vector3d nOrderVehicleTrajectory(int n, double t);
    bool isVehicleDeviateTrajectory(double threshold, geometry_msgs::Point vehicle_pos, double t);
    void printAll();
  };
}
#endif
