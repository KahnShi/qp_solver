#include <quadrotor_trajectory/QuadrotorCommand.h>

USING_NAMESPACE_QPOASES
namespace quadrotor_command
{
  void QuadrotorCommand::onInit()
  {
    for (int i = 0; i < 3; ++i)
      m_i_term_accumulation[i] = 0.0;
    m_control_freq = 50;
    m_takeoff_flag = 0;

    m_pub_uav_cmd = m_nh.advertise<geometry_msgs::Twist>(m_uav_cmd_pub_topic_name, 1);
    m_sub_truck_odom = m_nh.subscribe<nav_msgs::Odometry>(m_truck_odom_sub_topic_name, 1, &QuadrotorCommand::truckOdomCallback, this);
    m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>(m_uav_odom_sub_topic_name, 1, &QuadrotorCommand::uavOdomCallback, this);
    sleep(0.2); //To collect initial values for truck and uav odom, which will be used in following functions
  }

  void QuadrotorCommand::truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg)
  {
    // 50 hz in simulation
    m_truck_world_pos[0] = truck_odom_msg->pose.pose.position.x;
    m_truck_world_pos[1] = truck_odom_msg->pose.pose.position.y;
    m_truck_world_pos[2] = 0;

  }

  void QuadrotorCommand::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_odom_msg)
  {
    // 100 hz in simulation
    m_uav_world_pos[0] = uav_odom_msg->pose.pose.position.x;
    m_uav_world_pos[1] = uav_odom_msg->pose.pose.position.y;
    m_uav_world_pos[2] = uav_odom_msg->pose.pose.position.z;
    m_uav_q.x() = uav_odom_msg->pose.pose.orientation.x;
    m_uav_q.y() = uav_odom_msg->pose.pose.orientation.y;
    m_uav_q.z() = uav_odom_msg->pose.pose.orientation.z;
    m_uav_q.w() = uav_odom_msg->pose.pose.orientation.w;

    if (m_takeoff_flag == 0)
      {
        geometry_msgs::Twist uav_cmd;
        m_pub_uav_cmd.publish(uav_cmd);
        sleep(0.2);
        uav_cmd.linear.x = 0;
        uav_cmd.linear.y = 0;
        uav_cmd.linear.z = 1;
        m_pub_uav_cmd.publish(uav_cmd);
        sleep(0.2);
        m_pub_uav_cmd.publish(uav_cmd);
        std::cout << "Take off command is sent.\n";
        m_takeoff_flag = 1;
      }
    else if (m_takeoff_flag == 1)
      {
        if (m_uav_world_pos[2] > m_uav_initial_height)
          {
            geometry_msgs::Twist uav_cmd;
            uav_cmd.linear.x = 0;
            uav_cmd.linear.y = 0;
            uav_cmd.linear.z = 0;
            m_pub_uav_cmd.publish(uav_cmd);
            sleep(0.1);
            m_pub_uav_cmd.publish(uav_cmd);
            m_takeoff_flag = 2;
            std::cout << "Take off task is finished.\n";
          }
      }

  }

  bool QuadrotorCommand::isUavTruckNear(double threshold)
  {
    return (pow((m_uav_world_pos[0]-m_truck_world_pos[0]), 2) + pow((m_uav_world_pos[1]-m_truck_world_pos[1]), 2) < threshold);
  }

  void QuadrotorCommand::updateUavTruckRelPos()
  {
    m_uav_truck_world_pos = m_uav_world_pos - m_truck_world_pos;
  }

  void QuadrotorCommand::pidTracking()
  {
    //if (m_takeoff_flag < 2)
    //return;
    Vector3d truck_uav_rel_world_pos = m_truck_world_pos - m_uav_world_pos;
    Vector3d uav_euler = m_uav_q.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
    Matrix3d uav_rot_mat;
    uav_rot_mat = AngleAxisd(uav_euler[0], Vector3d::UnitZ())
      * AngleAxisd(0, Vector3d::UnitX())
      * AngleAxisd(0,  Vector3d::UnitY());
    std::cout << "Q: " << m_uav_q.x() << ' ' << m_uav_q.y() << ' ' << m_uav_q.z() << ' ' << m_uav_q.w() << "\n";
    std::cout << "Yaw axis: " << uav_euler[0] << "\n";
    Vector3d truck_uav_rel_uav_pos = uav_rot_mat.inverse() * truck_uav_rel_world_pos;

    Vector3d p_term = truck_uav_rel_uav_pos * m_p_gain;
    m_i_term_accumulation = m_i_term_accumulation + truck_uav_rel_uav_pos / m_control_freq;
    for (int i = 0; i < 3; ++i)
      {
        if (p_term[i] > m_p_term_max)
          p_term[i] = m_p_term_max;
        else if (p_term[i] < -m_p_term_max)
          p_term[i] = -m_p_term_max;

        if (m_i_term_accumulation[i] > m_i_term_max)
          m_i_term_accumulation[i] = m_i_term_max;
        else if (m_i_term_accumulation[i] < -m_i_term_max)
          m_i_term_accumulation[i] = -m_i_term_max;
      }
    Vector3d origin_cmd_vel = p_term + m_i_term_accumulation;

    // Currently only consider speed in x,y dim
    double origin_vel_norm = sqrt(pow(origin_cmd_vel[0], 2) + pow(origin_cmd_vel[1], 2));
    geometry_msgs::Twist uav_cmd;
    if (origin_vel_norm > m_uav_vel_ub)
      {
        uav_cmd.linear.x = origin_cmd_vel[0] * m_uav_vel_ub / origin_vel_norm;
        uav_cmd.linear.y = origin_cmd_vel[1] * m_uav_vel_ub / origin_vel_norm;
        uav_cmd.linear.z = 0;
      }
    else
      {
        uav_cmd.linear.x = origin_cmd_vel[0];
        uav_cmd.linear.y = origin_cmd_vel[1];
        uav_cmd.linear.z = 0;
      }
    m_pub_uav_cmd.publish(uav_cmd);

    std::cout << "Truck Uav dis: " << truck_uav_rel_uav_pos[0] << ", " << truck_uav_rel_uav_pos[1] << "\n";
    std::cout << "Uav cmd vel: " << uav_cmd.linear.x << ", " << uav_cmd.linear.y << "\n\n";
  }

}
