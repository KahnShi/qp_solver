
#include <quadrotor_trajectory/TruckTrajectoryEstimator.h>

USING_NAMESPACE_QPOASES
namespace truck_trajectory_estimator
{
  void TruckTrajectoryEstimator::onInit()
  {
    ros::NodeHandle pnh("~");
    /* ROS Node */
    pnh.param("truck_odom_sub_topic_name", m_truck_odom_sub_topic_name, std::string("/simulating_truck_odom"));
    pnh.param("truck_traj_polynomial_order", m_truck_traj_order, 6);
    pnh.param("truck_traj_derivation_order", m_truck_traj_dev_order, 2);
    pnh.param("truck_lambda_D", m_truck_lambda_D, 0.01);
    pnh.param("truck_estimate_odom_number", m_n_truck_estimate_odom, 40);
    pnh.param("truck_trajectory_generate_freqency", m_truck_traj_generate_freq, 60);
    pnh.param("truck_visualization_predict_time", m_truck_vis_predict_time, 2.0);
    pnh.param("truck_vis_predict_time_unit", m_truck_vis_predict_time_unit, 0.2);
    pnh.param("truck_smooth_forward_time", m_truck_smooth_forward_time, 1.0);
    pnh.param("truck_trajectory_deviation_threshold", m_truck_traj_deviation_threshold, 1.5);
    pnh.param("truck_max_velocity", m_truck_max_vel, 4.5);
    pnh.param("truck_max_acceleration", m_truck_max_acc, 1.5);

    pnh.param("uav_odom_sub_topic_name", m_uav_commander.m_uav_odom_sub_topic_name, std::string("/ground_truth/state"));
    pnh.param("uav_cmd_pub_topic_name", m_uav_commander.m_uav_cmd_pub_topic_name, std::string("/cmd_vel"));
    pnh.param("uav_vel_upper_bound", m_uav_commander.m_uav_vel_ub, 10.0);
    pnh.param("uav_vel_lower_bound", m_uav_commander.m_uav_vel_lb, -10.0);
    pnh.param("uav_acc_upper_bound", m_uav_commander.m_uav_acc_ub, 4.0);
    pnh.param("uav_acc_lower_bound", m_uav_commander.m_uav_acc_lb, -4.0);
    pnh.param("direct_pid_mode", m_uav_commander.m_direct_pid_mode, false);
    pnh.param("uav_cmd_direct_p_gain", m_uav_commander.m_direct_p_gain, 1.0);
    pnh.param("uav_cmd_direct_i_gain", m_uav_commander.m_direct_i_gain, 0.02);
    pnh.param("uav_cmd_direct_p_term_max", m_uav_commander.m_direct_p_term_max, 6.0);
    pnh.param("uav_cmd_direct_i_term_max", m_uav_commander.m_direct_i_term_max, 4.0);
    pnh.param("uav_initial_height", m_uav_commander.m_uav_initial_height, 6.0);
    pnh.param("uav_cmd_traj_track_p_gain", m_uav_commander.m_traj_track_p_gain, 1.0);
    pnh.param("uav_cmd_traj_track_i_gain", m_uav_commander.m_traj_track_i_gain, 0.0);
    pnh.param("uav_cmd_traj_track_d_gain", m_uav_commander.m_traj_track_d_gain, 1.0);
    pnh.param("uav_cmd_traj_track_p_term_max", m_uav_commander.m_traj_track_p_term_max, 5.0);
    pnh.param("uav_cmd_traj_track_i_term_max", m_uav_commander.m_traj_track_i_term_max, 3.0);
    pnh.param("uav_cmd_traj_track_d_term_max", m_uav_commander.m_traj_track_d_term_max, 3.0);

    if (m_uav_commander.m_direct_pid_mode)
      std::cout << "DIRECT PID MODE\n\n";
    else
      std::cout << "TRAJECTORY TRACKING MODE\n\n";

    m_server_ptr = boost::make_shared <dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig> > (pnh);
    dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig>::CallbackType f =
      boost::bind (&TruckTrajectoryEstimator::traj_estimate_config_callback, this, _1, _2);
    m_server_ptr->setCallback (f);

    m_n_current_odom = 0;
    m_n_truck_new_odom = 0;
    m_truck_odom_update = false;
    m_truck_odom_empty_flag = true;
    m_truck_odom_filled_flag = false;
    markerInit(m_truck_marker);
    m_truck_traj_param_x_ptr = new VectorXd(m_truck_traj_order);
    m_truck_traj_param_y_ptr = new VectorXd(m_truck_traj_order);

    // init for uav command
    m_uav_commander.m_truck_odom_sub_topic_name = m_truck_odom_sub_topic_name;
    m_uav_commander.onInit();
    m_uav_state = 0;

    m_sub_truck_odom = m_nh.subscribe<nav_msgs::Odometry>(m_truck_odom_sub_topic_name, 1, &TruckTrajectoryEstimator::truckOdomCallback, this);

    m_pub_truck_origin_path = m_nh.advertise<nav_msgs::Path>("/truck_path", 1);
    m_pub_truck_traj_path = m_nh.advertise<nav_msgs::Path>("/trajectory_path", 1);
    m_pub_uav_des_traj_path = m_nh.advertise<nav_msgs::Path>("/uav_des_traj_path", 1);
    m_pub_truck_origin_markers = m_nh.advertise<visualization_msgs::MarkerArray>("/truck_markers", 1);
    m_pub_truck_traj_markers = m_nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);


  }

  // 25 hz in simulation
  void TruckTrajectoryEstimator::truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg)
  {
    //ROS_INFO("Get Odom.");
    //std::cout << m_n_truck_new_odom << "\n";
    //uav command
    if (m_uav_state == 0)
      {
        if (m_uav_commander.m_takeoff_flag == 2)
          m_uav_state = 1;
        else
          return;
      }
    // Uav wait until it could "see" the truck.
    if (m_uav_state == 1)
      {
        if (m_uav_commander.isUavTruckNear(5.0))
          m_uav_state = 2;
        else
          return;
      }
    if (m_uav_state == 2)
      {
        if (m_uav_commander.isUavTruckNear(2.0) && m_truck_odom_filled_flag)
          {
            std::cout << "UAV is close to truck.\n";
            std::cout << "Starting trajectory tracking.\n";
            m_uav_state = 3;
            m_pub_uav_des_traj_path.publish(*m_uav_des_traj_path_ptr);
          }
        else
          m_uav_commander.directPidTracking();
      }
    if (m_uav_state == 3)
      {
        // Direct pid control
        if (m_uav_commander.m_direct_pid_mode)
          m_uav_commander.directPidTracking();
        // Trajectory tracking control
        else
          {
            double current_time = truck_odom_msg->header.stamp.toSec();
            Vector3d uav_des_pos = nOrderPolynomial(0, current_time) + Vector3d(m_uav_start_pos.getX(), m_uav_start_pos.getY(), m_uav_start_pos.getZ());
            Vector3d uav_des_vel = nOrderPolynomial(1, current_time);
            m_uav_commander.trajectoryTracking(uav_des_pos, uav_des_vel);
          }
      }

    nav_msgs::Odometry truck_odom = *truck_odom_msg;
    geometry_msgs::PoseStamped cur_truck_pose_stamped;
    cur_truck_pose_stamped.header = truck_odom.header;
    cur_truck_pose_stamped.pose = truck_odom.pose.pose;
    m_truck_marker.pose = truck_odom.pose.pose;
    m_truck_marker.id += 1;
    m_truck_marker.header = truck_odom.header;

    // Get first truck odom
    if (m_truck_odom_empty_flag)
      {
        // Init for variables in callback function
        m_truck_estimate_start_time = truck_odom.header.stamp.toSec();
        m_truck_odom_x_ptr = new VectorXd(m_n_truck_estimate_odom);
        m_truck_odom_y_ptr = new VectorXd(m_n_truck_estimate_odom);
        m_truck_odom_time_ptr = new VectorXd(m_n_truck_estimate_odom);
        m_truck_odom_empty_flag = false;
        m_truck_origin_path_ptr = new nav_msgs::Path();
        m_truck_origin_path_ptr->header = truck_odom.header;
        m_truck_origin_markers_ptr = new visualization_msgs::MarkerArray();
      }

    if (m_truck_odom_filled_flag)
      {
        // Erase the first element in array data
        for (int i = 0; i < m_n_truck_estimate_odom-1; ++i)
          {
            (*m_truck_odom_time_ptr)[i] = (*m_truck_odom_time_ptr)[i+1];
            (*m_truck_odom_x_ptr)[i] = (*m_truck_odom_x_ptr)[i+1];
            (*m_truck_odom_y_ptr)[i] = (*m_truck_odom_y_ptr)[i+1];
            m_truck_origin_path_ptr->header = m_truck_origin_path_ptr->poses[0].header;
            m_truck_origin_path_ptr->poses[i] = m_truck_origin_path_ptr->poses[i+1];
            m_truck_origin_markers_ptr->markers[i] = m_truck_origin_markers_ptr->markers[i+1];
          }

        m_truck_estimate_start_time = (*m_truck_odom_time_ptr)[0];

        (*m_truck_odom_time_ptr)[m_n_truck_estimate_odom-1] = truck_odom.header.stamp.toSec();
        (*m_truck_odom_x_ptr)[m_n_truck_estimate_odom-1] = truck_odom.pose.pose.position.x;
        (*m_truck_odom_y_ptr)[m_n_truck_estimate_odom-1] = truck_odom.pose.pose.position.y;
        m_truck_origin_path_ptr->poses[m_n_truck_estimate_odom-1] = cur_truck_pose_stamped;
        m_truck_origin_markers_ptr->markers[m_n_truck_estimate_odom-1] = m_truck_marker;

        ++m_n_truck_new_odom;
        if (m_n_truck_new_odom >= m_truck_traj_generate_freq || isTruckDeviateTrajectory(m_truck_traj_deviation_threshold, truck_odom.pose.pose.position, (*m_truck_odom_time_ptr)[m_n_truck_estimate_odom-1]))
          {
            m_n_truck_new_odom = 0;
            delete m_truck_traj_path_ptr;
            m_truck_traj_path_ptr = new nav_msgs::Path();
            m_truck_traj_path_ptr->header = m_truck_origin_path_ptr->header;
            delete m_uav_des_traj_path_ptr;
            m_uav_des_traj_path_ptr = new nav_msgs::Path();
            m_uav_des_traj_path_ptr->header = m_truck_origin_path_ptr->header;
            m_truck_traj_start_time = m_truck_estimate_start_time;
            ROS_INFO("Start truck traj polynomial estmate.");
            polynomialEstimation();
            ROS_INFO("Finish truck traj polynomial estmate.");
            trajectoryVisualization();
          }
        else
          {
          }
      }
    // Truck's trajectory is estimated after enough odom is got.
    else
      {
        (*m_truck_odom_time_ptr)[m_n_current_odom] = truck_odom.header.stamp.toSec();
        (*m_truck_odom_x_ptr)[m_n_current_odom] = truck_odom.pose.pose.position.x;
        (*m_truck_odom_y_ptr)[m_n_current_odom] = truck_odom.pose.pose.position.y;
        m_truck_origin_path_ptr->poses.push_back(cur_truck_pose_stamped);
        m_truck_origin_markers_ptr->markers.push_back(m_truck_marker);

        ++m_n_current_odom;
        if (m_n_current_odom >= m_n_truck_estimate_odom)
          {
            m_truck_odom_filled_flag = true;
            m_truck_traj_path_ptr = new nav_msgs::Path();
            m_truck_traj_path_ptr->header = m_truck_origin_path_ptr->header;
            m_uav_des_traj_path_ptr = new nav_msgs::Path();
            m_uav_des_traj_path_ptr->header = m_truck_origin_path_ptr->header;
            polynomialEstimation();
            trajectoryVisualization();
          }
      }
    m_pub_truck_origin_path.publish(*m_truck_origin_path_ptr);
    m_pub_truck_origin_markers.publish(*m_truck_origin_markers_ptr);

    //ROS_INFO("Finish Odom.");
  }

  void TruckTrajectoryEstimator::polynomialEstimation()
  {
    MatrixXd T = MatrixXd::Zero(m_truck_traj_order, m_truck_traj_order);
    MatrixXd D = MatrixXd::Zero(m_truck_traj_order, m_truck_traj_order);
    MatrixXd H = MatrixXd::Zero(m_truck_traj_order, m_truck_traj_order);
    VectorXd g_x = VectorXd::Zero(m_truck_traj_order);
    VectorXd g_y = VectorXd::Zero(m_truck_traj_order);

    for (int point_index = 0; point_index < m_n_truck_estimate_odom; ++point_index)
      {
        VectorXd t_t = VectorXd::Zero(m_truck_traj_order);
        //t_t_d is t_t after derivation_order operation
        VectorXd t_t_d = VectorXd::Zero(m_truck_traj_order);
        t_t[0] = 1;
        t_t_d[m_truck_traj_dev_order] = 1;
        for (int i = 1; i < m_truck_traj_order; ++i)
          {
            t_t[i] = t_t[i-1] * ((*m_truck_odom_time_ptr)[point_index] - m_truck_estimate_start_time);
            if (i >= m_truck_traj_dev_order)
              //t_t_d[i] = t_t[i-m_truck_traj_dev_order];
              t_t_d[i] = t_t_d[i-1] * ((*m_truck_odom_time_ptr)[point_index] + m_truck_smooth_forward_time - m_truck_estimate_start_time);
          }
        T = T + t_t * t_t.transpose();
        D = D + t_t_d * t_t_d.transpose();
        g_x = g_x + (-2) * t_t * (*m_truck_odom_x_ptr)[point_index];
        g_y = g_y + (-2) * t_t * (*m_truck_odom_y_ptr)[point_index];
      }

    // Add constraints
    // Check velocity and acceleration in future 0~2 second, and every 0.2 second
    int n_constraints = int(3.0/0.1);
    MatrixXd A = MatrixXd::Zero(n_constraints*2, m_truck_traj_order);
    VectorXd lb_A = VectorXd::Zero(n_constraints*2);
    VectorXd ub_A = VectorXd::Zero(n_constraints*2);
    for (int i = 0; i < n_constraints; ++i)
      {
        double cur_t = 0.2 * i;
        double mul = cur_t;
        A(2*i, 1) = 1;
        A(2*i, 2) = 2*cur_t;
        A(2*i+1, 2) = 2;
        for (int j = 3; j < m_truck_traj_order; ++j)
          {
            A(2*i+1, j) = j * (j - 1) * mul;
            mul *= cur_t;
            A(2*i, j) = j * mul;
          }
        lb_A(2*i) = -m_truck_max_vel;
        lb_A(2*i+1) = -m_truck_max_acc;
        ub_A(2*i) = m_truck_max_vel;
        ub_A(2*i+1) = m_truck_max_acc;
      }

    // Assign value for H matrix
    H = 2 * T + m_truck_lambda_D * m_n_truck_estimate_odom * D;

    /* Setting up QProblemB object. */

    QProblem exampleQ_x( m_truck_traj_order,0 );
    QProblem exampleQ_y( m_truck_traj_order,0 );

    Options options;
    //options.enableFlippingBounds = BT_FALSE;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;

    // Bakui's
    options.enableEqualities = BT_TRUE;
    options.printLevel = PL_LOW;
    exampleQ_x.setOptions( options );
    exampleQ_y.setOptions( options );

    int_t nWSR_x = 10;
    exampleQ_x.init(H.data(),g_x.data(),A.data(),NULL,NULL,lb_A.data(), ub_A.data(), nWSR_x,0 );
    exampleQ_x.getPrimalSolution(m_truck_traj_param_x_ptr->data());

    int_t nWSR_y = 10;
    exampleQ_y.init(H.data(),g_y.data(),A.data(),NULL,NULL,lb_A.data(), ub_A.data(), nWSR_y,0 );
    exampleQ_y.getPrimalSolution(m_truck_traj_param_y_ptr->data());
    //printf( " ];  objVal = %e\n\n", exampleQ.getObjVal() );

    // std::cout <<"[x]: ";
    // for (int i = 0; i < m_truck_traj_order; ++i)
    //   std::cout << truck_traj_param_x_->data()[i] << ", ";
    // printf("\n");

  }

  void TruckTrajectoryEstimator::trajectoryVisualization()
  {
    for (int point_id = 0; point_id < m_n_truck_estimate_odom; ++point_id)
      {
        geometry_msgs::PoseStamped cur_pose;
        double delta_t = (*m_truck_odom_time_ptr)[point_id] - m_truck_traj_start_time;
        cur_pose = m_truck_origin_path_ptr->poses[point_id];
        cur_pose.pose.position.x = getPointFromPolynomial('x', delta_t);
        cur_pose.pose.position.y = getPointFromPolynomial('y', delta_t);
        m_truck_traj_path_ptr->poses.push_back(cur_pose);
      }

    m_uav_commander.updateUavTruckRelPos();
    m_uav_start_pos = m_uav_commander.m_uav_truck_world_pos;

    int predict_number = int(m_truck_vis_predict_time / m_truck_vis_predict_time_unit);
    double delta_t_offset = (*m_truck_odom_time_ptr)[m_n_truck_estimate_odom-1] - m_truck_traj_start_time;
    geometry_msgs::PoseStamped last_pose = m_truck_origin_path_ptr->poses[m_n_truck_estimate_odom-1];
    for (int point_id = 0; point_id < predict_number; ++point_id)
      {
        geometry_msgs::PoseStamped cur_pose;
        double delta_t = delta_t_offset + point_id * m_truck_vis_predict_time_unit;
        cur_pose = last_pose;
        cur_pose.pose.position.x = getPointFromPolynomial('x', delta_t);
        cur_pose.pose.position.y = getPointFromPolynomial('y', delta_t);
        m_truck_traj_path_ptr->poses.push_back(cur_pose);
        // Uav's predicted traj = offset + truck's predicted traj
        cur_pose.pose.position.x += m_uav_start_pos.getX();
        cur_pose.pose.position.y += m_uav_start_pos.getY();
        cur_pose.pose.position.z += m_uav_start_pos.getZ();
        m_uav_des_traj_path_ptr->poses.push_back(cur_pose);
      }
    m_pub_truck_traj_path.publish(*m_truck_traj_path_ptr);
    if (m_uav_state == 3)
      m_pub_uav_des_traj_path.publish(*m_uav_des_traj_path_ptr);
  }

  void TruckTrajectoryEstimator::markerInit(visualization_msgs::Marker &marker, char color)
  {
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0.0;
    marker.color.b = 0;
    marker.lifetime = ros::Duration(2.0);
    if (color == 'r')
      marker.color.r = 1;
    else if (color == 'g')
      marker.color.g = 1;
    else if (color == 'b')
      marker.color.b = 1;
  }
  int TruckTrajectoryEstimator::factorial(int n, int order)
  {
    int result = 1;
    for (int i = 0; i < order; ++i)
      result *= (n - i);
    return result;
  }

  double TruckTrajectoryEstimator::getPointFromPolynomial(char axis, double var_value)
  {
    double result = 0, temp = 1;
    for (int i = 0; i < m_truck_traj_order; ++i)
      {
        if (axis == 'x')
          result += (*m_truck_traj_param_x_ptr)[i] * temp;
        else if (axis == 'y')
          result += (*m_truck_traj_param_y_ptr)[i] * temp;
        temp *= var_value;
      }
    return result;
  }


  Vector3d TruckTrajectoryEstimator::nOrderPolynomial(int n, double t)
  {
    double temp = 1, delta_t = t-m_truck_estimate_start_time;
    Vector3d result(0.0, 0.0, 0.0);
    for (int i = n; i < m_truck_traj_order; ++i)
      {
        int factor = factorial(i, n);
        result[0] += factor * (*m_truck_traj_param_x_ptr)[i] * temp;
        result[1] += factor * (*m_truck_traj_param_y_ptr)[i] * temp;
        temp *= delta_t;
      }
    return result;
  }

  void TruckTrajectoryEstimator::traj_estimate_config_callback(quadrotor_trajectory::TrajectoryEstimateConfig &config, uint32_t level)
  {
    if (config.enable)
      {
        ROS_WARN("new config");
        m_truck_traj_order = config.truck_traj_order;
        m_truck_traj_dev_order = config.derivation_order;
        m_truck_lambda_D= config.truck_lambda_D;
        m_n_truck_estimate_odom = config.truck_estimate_odom_number;
        m_truck_traj_generate_freq = config.truck_trajectory_generate_frequency;
        m_truck_smooth_forward_time = config.truck_smooth_forward_time;
        m_truck_vis_predict_time = config.truck_visualization_predict_time;
        m_uav_commander.m_direct_pid_mode = config.direct_pid_mode;
        m_uav_commander.m_traj_track_p_gain = config.traj_track_p_param;
        m_uav_commander.m_traj_track_i_gain = config.traj_track_i_param;
        m_uav_commander.m_traj_track_d_gain = config.traj_track_d_param;
      }
  }

  bool TruckTrajectoryEstimator::isTruckDeviateTrajectory(double threshold, geometry_msgs::Point truck_pos, double current_time)
  {
    double delta_t = current_time-m_truck_traj_start_time;
    double distance = pow(getPointFromPolynomial('x', delta_t)-truck_pos.x, 2) + pow(getPointFromPolynomial('y', delta_t)-truck_pos.y, 2);
    if (distance > pow(threshold, 2))
      {
        ROS_WARN("Truck is deviate from predict trajectoy.");
        std::cout << "Deviation is : " << sqrt(distance) << ", threshold is : " << threshold << "\n";
        return true;
      }
    else
      return false;
  }
}
