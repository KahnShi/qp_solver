#include <quadrotor_trajectory/QuadrotorCommand.h>

USING_NAMESPACE_QPOASES
namespace quadrotor_command
{
  void QuadrotorCommand::onInit()
  {
    m_sub_truck_odom = m_nh.subscribe<nav_msgs::Odometry>(m_truck_odom_sub_topic_name, 1, &QuadrotorCommand::truckOdomCallback, this);
    m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>(m_uav_odom_sub_topic_name, 1, &QuadrotorCommand::uavOdomCallback, this);
  }

  void QuadrotorCommand::truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg)
  {
    // 50 hz in simulation
    m_truck_world_pos[0] = truck_odom_msg->pose.pose.position.x;
    m_truck_world_pos[1] = truck_odom_msg->pose.pose.position.y;

  }

  void QuadrotorCommand::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_odom_msg)
  {
    // 100 hz in simulation
    m_uav_world_pos[0] = uav_odom_msg->pose.pose.position.x;
    m_uav_world_pos[1] = uav_odom_msg->pose.pose.position.y;
    m_uav_q[0] = uav_odom_msg->pose.pose.orientation.x;
    m_uav_q[1] = uav_odom_msg->pose.pose.orientation.y;
    m_uav_q[2] = uav_odom_msg->pose.pose.orientation.z;
    m_uav_q[3] = uav_odom_msg->pose.pose.orientation.w;
  }

  bool QuadrotorCommand::isUavTruckNear(double threshold)
  {
    return (pow((m_uav_world_pos[0]-m_truck_world_pos[0]), 2) + pow((m_uav_world_pos[1]-m_truck_world_pos[1]), 2) < threshold);
  }
}
