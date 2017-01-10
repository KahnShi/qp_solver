#include <quadrotor_trajectory/QuadrotorCommand.h>

USING_NAMESPACE_QPOASES
namespace quadrotor_command
{
  void QuadrotorCommand::onInit()
  {
    m_direct_i_term_accumulation.setValue(0.0, 0.0, 0.0);
    m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
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
    m_truck_world_pos.setValue(truck_odom_msg->pose.pose.position.x,
                               truck_odom_msg->pose.pose.position.y,
                               0.0);

  }

  void QuadrotorCommand::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_odom_msg)
  {
    // 100 hz in simulation
    m_uav_world_pos.setValue(uav_odom_msg->pose.pose.position.x,
                             uav_odom_msg->pose.pose.position.y,
                             uav_odom_msg->pose.pose.position.z);
    m_uav_world_vel.setValue(uav_odom_msg->twist.twist.linear.x,
                             uav_odom_msg->twist.twist.linear.y,
                             uav_odom_msg->twist.twist.linear.z);
    m_uav_q = tf::Quaternion(uav_odom_msg->pose.pose.orientation.x,
                             uav_odom_msg->pose.pose.orientation.y,
                             uav_odom_msg->pose.pose.orientation.z,
                             uav_odom_msg->pose.pose.orientation.w);

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
        else
          {
            geometry_msgs::Twist uav_cmd;
            uav_cmd.linear.x = 0;
            uav_cmd.linear.y = 0;
            uav_cmd.linear.z = 1;
            m_pub_uav_cmd.publish(uav_cmd);
          }
      }

  }

  bool QuadrotorCommand::isUavTruckNear(double threshold)
  {
    return (pow((m_uav_world_pos.getX()-m_truck_world_pos.getX()), 2) + pow((m_uav_world_pos.getY()-m_truck_world_pos.getY()), 2) < threshold);
  }

  void QuadrotorCommand::updateUavTruckRelPos()
  {
    m_uav_truck_world_pos = m_uav_world_pos - m_truck_world_pos;
  }

  void QuadrotorCommand::directPidTracking()
  {
    //if (m_takeoff_flag < 2)
    //return;
    tf::Vector3 truck_uav_rel_world_pos = m_truck_world_pos - m_uav_world_pos;
    tf::Matrix3x3  uav_rot_mat(m_uav_q);
    tfScalar uav_roll, uav_pitch, uav_yaw;
    uav_rot_mat.getRPY(uav_roll, uav_pitch, uav_yaw);
    tf::Matrix3x3 r_z;
    r_z.setRPY(0, 0, uav_yaw);
    std::cout << "Yaw angle: " << uav_yaw / 3.14 * 180.0 << " deg\n";
    tf::Vector3 truck_uav_rel_uav_pos = r_z.inverse() * truck_uav_rel_world_pos;

    tf::Vector3 direct_p_term = truck_uav_rel_uav_pos * m_direct_p_gain;
    m_direct_i_term_accumulation = m_direct_i_term_accumulation + truck_uav_rel_uav_pos / m_control_freq;
    if (direct_p_term.getX() > m_direct_p_term_max) direct_p_term.setX(m_direct_p_term_max);
    else if (direct_p_term.getX() < -m_direct_p_term_max) direct_p_term.setX(-m_direct_p_term_max);
    if (direct_p_term.getY() > m_direct_p_term_max) direct_p_term.setY(m_direct_p_term_max);
    else if (direct_p_term.getY() < -m_direct_p_term_max) direct_p_term.setY(-m_direct_p_term_max);
    if (direct_p_term.getZ() > m_direct_p_term_max) direct_p_term.setZ(m_direct_p_term_max);
    else if (direct_p_term.getZ() < -m_direct_p_term_max) direct_p_term.setZ(-m_direct_p_term_max);

    if (m_direct_i_term_accumulation.getX() > m_direct_i_term_max) m_direct_i_term_accumulation.setX(m_direct_i_term_max);
    else if (m_direct_i_term_accumulation.getX() < -m_direct_i_term_max) m_direct_i_term_accumulation.setX(-m_direct_i_term_max);
    if (m_direct_i_term_accumulation.getY() > m_direct_i_term_max) m_direct_i_term_accumulation.setY(m_direct_i_term_max);
    else if (m_direct_i_term_accumulation.getY() < -m_direct_i_term_max) m_direct_i_term_accumulation.setY(-m_direct_i_term_max);
    if (m_direct_i_term_accumulation.getZ() > m_direct_i_term_max) m_direct_i_term_accumulation.setZ(m_direct_i_term_max);
    else if (m_direct_i_term_accumulation.getZ() < -m_direct_i_term_max) m_direct_i_term_accumulation.setZ(-m_direct_i_term_max);

    std::cout << "P term: " << direct_p_term.getX()<<' ' << direct_p_term.getY() << "\n";
    std::cout << "I term: " << m_direct_i_term_accumulation.getX()<<' ' << m_direct_i_term_accumulation.getY() << "\n";
    tf::Vector3 origin_cmd_vel = direct_p_term + m_direct_i_term_accumulation;

    // Currently only consider speed in x,y dim
    double origin_vel_norm = sqrt(pow(origin_cmd_vel.getX(), 2) + pow(origin_cmd_vel.getY(), 2));
    geometry_msgs::Twist uav_cmd;
    if (origin_vel_norm > m_uav_vel_ub)
      {
        uav_cmd.linear.x = origin_cmd_vel.getX() * m_uav_vel_ub / origin_vel_norm;
        uav_cmd.linear.y = origin_cmd_vel.getY() * m_uav_vel_ub / origin_vel_norm;
        uav_cmd.linear.z = 0;
      }
    else
      {
        uav_cmd.linear.x = origin_cmd_vel.getX();
        uav_cmd.linear.y = origin_cmd_vel.getY();
        uav_cmd.linear.z = 0;
      }
    m_pub_uav_cmd.publish(uav_cmd);

    std::cout << "Truck Uav dis: " << truck_uav_rel_uav_pos.getX() << ", " << truck_uav_rel_uav_pos.getY() << "\n";
    std::cout << "Uav cmd vel: " << uav_cmd.linear.x << ", " << uav_cmd.linear.y << "\n\n";
  }


  void QuadrotorCommand::trajectoryTracking(Vector3d uav_des_pos, Vector3d uav_des_vel)
  {
    tf::Vector3 des_uav_world_pos(uav_des_pos[0], uav_des_pos[1], uav_des_pos[2]);
    tf::Vector3 des_uav_world_vel(uav_des_vel[0], uav_des_vel[1], uav_des_vel[2]);
    tf::Vector3 des_uav_rel_world_pos = des_uav_world_pos - m_uav_world_pos;
    tf::Vector3 des_uav_rel_world_vel = des_uav_world_vel - m_uav_world_vel;
    tf::Matrix3x3  uav_rot_mat(m_uav_q);
    tfScalar uav_roll, uav_pitch, uav_yaw;
    uav_rot_mat.getRPY(uav_roll, uav_pitch, uav_yaw);
    tf::Matrix3x3 r_z, r_z_inv;
    r_z.setRPY(0, 0, uav_yaw);
    r_z_inv = r_z.inverse();
    std::cout << "Yaw angle: " << uav_yaw / 3.14 * 180.0 << " deg\n";
    tf::Vector3 des_uav_rel_uav_pos = r_z_inv * des_uav_rel_world_pos;
    tf::Vector3 des_uav_rel_uav_vel = r_z_inv * des_uav_rel_world_vel;

    // p term
    tf::Vector3 traj_track_p_term = des_uav_rel_uav_pos * m_traj_track_p_gain;
    if (traj_track_p_term.getX() > m_traj_track_p_term_max) traj_track_p_term.setX(m_traj_track_p_term_max);
    else if (traj_track_p_term.getX() < -m_traj_track_p_term_max) traj_track_p_term.setX(-m_traj_track_p_term_max);
    if (traj_track_p_term.getY() > m_traj_track_p_term_max) traj_track_p_term.setY(m_traj_track_p_term_max);
    else if (traj_track_p_term.getY() < -m_traj_track_p_term_max) traj_track_p_term.setY(-m_traj_track_p_term_max);
    if (traj_track_p_term.getZ() > m_traj_track_p_term_max) traj_track_p_term.setZ(m_traj_track_p_term_max);
    else if (traj_track_p_term.getZ() < -m_traj_track_p_term_max) traj_track_p_term.setZ(-m_traj_track_p_term_max);

    // I term
    m_traj_track_i_term_accumulation = m_traj_track_i_term_accumulation + des_uav_rel_uav_pos / m_control_freq;
    if (m_traj_track_i_term_accumulation.getX() > m_traj_track_i_term_max) m_traj_track_i_term_accumulation.setX(m_traj_track_i_term_max);
    else if (m_traj_track_i_term_accumulation.getX() < -m_traj_track_i_term_max) m_traj_track_i_term_accumulation.setX(-m_traj_track_i_term_max);
    if (m_traj_track_i_term_accumulation.getY() > m_traj_track_i_term_max) m_traj_track_i_term_accumulation.setY(m_traj_track_i_term_max);
    else if (m_traj_track_i_term_accumulation.getY() < -m_traj_track_i_term_max) m_traj_track_i_term_accumulation.setY(-m_traj_track_i_term_max);
    if (m_traj_track_i_term_accumulation.getZ() > m_traj_track_i_term_max) m_traj_track_i_term_accumulation.setZ(m_traj_track_i_term_max);
    else if (m_traj_track_i_term_accumulation.getZ() < -m_traj_track_i_term_max) m_traj_track_i_term_accumulation.setZ(-m_traj_track_i_term_max);
    tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * m_traj_track_i_gain;

    // D term
    tf::Vector3 traj_track_d_term = des_uav_rel_uav_vel * m_traj_track_d_gain;

    std::cout << "P term: " << traj_track_p_term.getX()<<' ' << traj_track_p_term.getY() << "\n";
    std::cout << "I term: " << m_traj_track_i_term_accumulation.getX()<<' ' << m_traj_track_i_term_accumulation.getY() << "\n";
    std::cout << "D term: " << traj_track_d_term.getX()<<' ' << traj_track_d_term.getY() << "\n";

    // pid
    tf::Vector3 origin_cmd_vel = traj_track_p_term + traj_track_d_term + traj_track_i_term;
    std::cout << "Origin cmd vel: " << origin_cmd_vel.getX() << ", " << origin_cmd_vel.getY() << "\n";
    // Currently only consider speed in x,y dim
    double origin_vel_norm = sqrt(pow(origin_cmd_vel.getX(), 2) + pow(origin_cmd_vel.getY(), 2));
    geometry_msgs::Twist uav_cmd;
    if (origin_vel_norm > m_uav_vel_ub)
      {
        std::cout << "UAV cmd vel is above upper bound. \n";
        uav_cmd.linear.x = origin_cmd_vel.getX() * m_uav_vel_ub / origin_vel_norm;
        uav_cmd.linear.y = origin_cmd_vel.getY() * m_uav_vel_ub / origin_vel_norm;
        uav_cmd.linear.z = 0;
      }
    else
      {
        uav_cmd.linear.x = origin_cmd_vel.getX();
        uav_cmd.linear.y = origin_cmd_vel.getY();
        uav_cmd.linear.z = 0;
      }
    m_pub_uav_cmd.publish(uav_cmd);

    std::cout << "Desired Uav pos dis: " << des_uav_rel_uav_pos.getX() << ", " << des_uav_rel_uav_pos.getY() << "\n";
    std::cout << "Desired Uav vel dis: " << des_uav_rel_uav_vel.getX() << ", " << des_uav_rel_uav_vel.getY() << "\n";
    std::cout << "Uav cmd vel: " << uav_cmd.linear.x << ", " << uav_cmd.linear.y << "\n\n";
  }

}