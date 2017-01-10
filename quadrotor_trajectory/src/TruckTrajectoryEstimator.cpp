
#include <quadrotor_trajectory/TruckTrajectoryEstimator.h>

USING_NAMESPACE_QPOASES
namespace truck_trajectory_estimator
{
  void TruckTrajectoryEstimator::onInit()
  {
    ros::NodeHandle pnh_("~");
    /* ROS Node */
    pnh_.param("truck_odom_sub_topic_name", truck_odom_sub_topic_name_, std::string("/simulating_truck_odom"));
    pnh_.param("polynomial_order", polynomial_order_, 6);
    pnh_.param("derivation_order", derivation_order_, 2);
    pnh_.param("lambda_D", lambda_D_, 0.01);
    pnh_.param("estimating_odom_number", estimating_odom_number_, 40);
    pnh_.param("trajectory_generate_freqency", traj_generate_freq_, 60);
    pnh_.param("visualization_predict_time", visualization_predict_time_, 2.0);
    pnh_.param("visualization_predict_time_unit", visualization_predict_time_unit_, 0.2);
    pnh_.param("smooth_forward_time", smooth_forward_time_, 1.0);
    pnh_.param("uav_odom_sub_topic_name", uav_commander_.m_uav_odom_sub_topic_name, std::string("/ground_truth/state"));
    pnh_.param("uav_cmd_pub_topic_name", uav_commander_.m_uav_cmd_pub_topic_name, std::string("/cmd_vel"));
    pnh_.param("uav_vel_upper_bound", uav_commander_.m_uav_vel_ub, 10.0);
    pnh_.param("uav_vel_lower_bound", uav_commander_.m_uav_vel_lb, -10.0);
    pnh_.param("uav_acc_upper_bound", uav_commander_.m_uav_acc_ub, 4.0);
    pnh_.param("uav_acc_lower_bound", uav_commander_.m_uav_acc_lb, -4.0);
    pnh_.param("direct_pid_mode", uav_commander_.m_direct_pid_mode, false);
    pnh_.param("uav_cmd_direct_p_gain", uav_commander_.m_direct_p_gain, 1.0);
    pnh_.param("uav_cmd_direct_i_gain", uav_commander_.m_direct_i_gain, 0.02);
    pnh_.param("uav_cmd_direct_p_term_max", uav_commander_.m_direct_p_term_max, 6.0);
    pnh_.param("uav_cmd_direct_i_term_max", uav_commander_.m_direct_i_term_max, 4.0);
    pnh_.param("uav_initial_height", uav_commander_.m_uav_initial_height, 6.0);
    pnh_.param("uav_cmd_traj_track_p_gain", uav_commander_.m_traj_track_p_gain, 1.0);
    pnh_.param("uav_cmd_traj_track_i_gain", uav_commander_.m_traj_track_i_gain, 0.0);
    pnh_.param("uav_cmd_traj_track_d_gain", uav_commander_.m_traj_track_d_gain, 1.0);
    pnh_.param("uav_cmd_traj_track_p_term_max", uav_commander_.m_traj_track_p_term_max, 5.0);
    pnh_.param("uav_cmd_traj_track_i_term_max", uav_commander_.m_traj_track_i_term_max, 3.0);
    pnh_.param("uav_cmd_traj_track_d_term_max", uav_commander_.m_traj_track_d_term_max, 3.0);

    if (uav_commander_.m_direct_pid_mode)
      std::cout << "DIRECT PID MODE\n\n";
    else
      std::cout << "TRAJECTORY TRACKING MODE\n\n";

    server_ptr_ = boost::make_shared <dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig> > (pnh_);
    dynamic_reconfigure::Server<quadrotor_trajectory::TrajectoryEstimateConfig>::CallbackType f =
      boost::bind (&TruckTrajectoryEstimator::traj_estimate_config_callback, this, _1, _2);
    server_ptr_->setCallback (f);

    current_odom_number_ = 0;
    new_traj_points_number_ = 0;
    truck_odom_update_ = false;
    truck_odom_empty_flag_ = true;
    truck_odom_filled_flag_ = false;
    markerInit(truck_marker_);
    truck_traj_param_x_ = new VectorXd(polynomial_order_);
    truck_traj_param_y_ = new VectorXd(polynomial_order_);

    // init for uav command
    uav_commander_.m_truck_odom_sub_topic_name = truck_odom_sub_topic_name_;
    uav_commander_.onInit();
    uav_state_ = 0;

    sub_truck_odom_ = nh_.subscribe<nav_msgs::Odometry>(truck_odom_sub_topic_name_, 1, &TruckTrajectoryEstimator::truckOdomCallback, this);

    pub_truck_origin_path_ = nh_.advertise<nav_msgs::Path>("/truck_path", 1);
    pub_truck_traj_path_ = nh_.advertise<nav_msgs::Path>("/trajectory_path", 1);
    pub_uav_des_traj_path_ = nh_.advertise<nav_msgs::Path>("/uav_des_traj_path", 1);
    pub_truck_origin_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/truck_markers", 1);
    pub_truck_traj_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);


  }


  void TruckTrajectoryEstimator::truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg)
  {
    //uav command
    if (uav_state_ == 0)
      {
        if (uav_commander_.m_takeoff_flag == 2)
          uav_state_ = 1;
        else
          return;
      }
    // Uav wait until it could "see" the truck.
    if (uav_state_ == 1)
      {
        if (uav_commander_.isUavTruckNear(5.0))
          uav_state_ = 2;
        else
          return;
      }
    if (uav_state_ == 2)
      {
        if (uav_commander_.isUavTruckNear(2.0) && truck_odom_filled_flag_)
          {
            std::cout << "UAV is close to truck.\n";
            std::cout << "Starting trajectory tracking.\n";
            uav_state_ = 3;
            pub_uav_des_traj_path_.publish(*uav_des_traj_path_);
          }
        else
          uav_commander_.directPidTracking();
      }
    if (uav_state_ == 3)
      {
        // Direct pid control
        if (uav_commander_.m_direct_pid_mode)
          uav_commander_.directPidTracking();
        // Trajectory tracking control
        else
          {
            double current_time = truck_odom_msg->header.stamp.toSec();
            Vector3d uav_des_pos = nOrderPolynomial(0, current_time) + Vector3d(uav_start_pos_.getX(), uav_start_pos_.getY(), uav_start_pos_.getZ());
            Vector3d uav_des_vel = nOrderPolynomial(1, current_time);
            uav_commander_.trajectoryTracking(uav_des_pos, uav_des_vel);
          }
      }

    nav_msgs::Odometry truck_odom = *truck_odom_msg;
    geometry_msgs::PoseStamped cur_truck_pose_stamped;
    cur_truck_pose_stamped.header = truck_odom.header;
    cur_truck_pose_stamped.pose = truck_odom.pose.pose;
    truck_marker_.pose = truck_odom.pose.pose;
    truck_marker_.id += 1;
    truck_marker_.header = truck_odom.header;

    // Get first truck odom
    if (truck_odom_empty_flag_)
      {
        // Init for variables in callback function
        estimating_start_time_ = truck_odom.header.stamp.toSec();
        truck_odom_x_ = new VectorXd(estimating_odom_number_);
        truck_odom_y_ = new VectorXd(estimating_odom_number_);
        truck_odom_time_ = new VectorXd(estimating_odom_number_);
        truck_odom_empty_flag_ = false;
        truck_origin_path_ = new nav_msgs::Path();
        truck_origin_path_->header = truck_odom.header;
        truck_origin_markers_ = new visualization_msgs::MarkerArray();
      }

    if (truck_odom_filled_flag_)
      {
        // Erase the first element in array data
        for (int i = 0; i < estimating_odom_number_-1; ++i)
          {
            (*truck_odom_time_)[i] = (*truck_odom_time_)[i+1];
            (*truck_odom_x_)[i] = (*truck_odom_x_)[i+1];
            (*truck_odom_y_)[i] = (*truck_odom_y_)[i+1];
            truck_origin_path_->header = truck_origin_path_->poses[0].header;
            truck_origin_path_->poses[i] = truck_origin_path_->poses[i+1];
            truck_origin_markers_->markers[i] = truck_origin_markers_->markers[i+1];
          }

        estimating_start_time_ = (*truck_odom_time_)[0];

        (*truck_odom_time_)[estimating_odom_number_-1] = truck_odom.header.stamp.toSec();
        (*truck_odom_x_)[estimating_odom_number_-1] = truck_odom.pose.pose.position.x;
        (*truck_odom_y_)[estimating_odom_number_-1] = truck_odom.pose.pose.position.y;
        truck_origin_path_->poses[estimating_odom_number_-1] = cur_truck_pose_stamped;
        truck_origin_markers_->markers[estimating_odom_number_-1] = truck_marker_;

        ++new_traj_points_number_;
        if (new_traj_points_number_ >= traj_generate_freq_)
          {
            new_traj_points_number_ = 0;
            delete truck_traj_path_;
            truck_traj_path_ = new nav_msgs::Path();
            truck_traj_path_->header = truck_origin_path_->header;
            delete uav_des_traj_path_;
            uav_des_traj_path_ = new nav_msgs::Path();
            uav_des_traj_path_->header = truck_origin_path_->header;
            trajectory_start_time_ = estimating_start_time_;
            polynomialEstimation();
            trajectoryVisualization();
          }
        else
          {
          }
      }
    // Truck's trajectory is estimated after enough odom is got.
    else
      {
        (*truck_odom_time_)[current_odom_number_] = truck_odom.header.stamp.toSec();
        (*truck_odom_x_)[current_odom_number_] = truck_odom.pose.pose.position.x;
        (*truck_odom_y_)[current_odom_number_] = truck_odom.pose.pose.position.y;
        truck_origin_path_->poses.push_back(cur_truck_pose_stamped);
        truck_origin_markers_->markers.push_back(truck_marker_);

        ++current_odom_number_;
        if (current_odom_number_ >= estimating_odom_number_)
          {
            truck_odom_filled_flag_ = true;
            truck_traj_path_ = new nav_msgs::Path();
            truck_traj_path_->header = truck_origin_path_->header;
            uav_des_traj_path_ = new nav_msgs::Path();
            uav_des_traj_path_->header = truck_origin_path_->header;
            polynomialEstimation();
            trajectoryVisualization();
          }
      }
    pub_truck_origin_path_.publish(*truck_origin_path_);
    pub_truck_origin_markers_.publish(*truck_origin_markers_);
  }

  void TruckTrajectoryEstimator::polynomialEstimation()
  {
    MatrixXd T = MatrixXd::Zero(polynomial_order_, polynomial_order_);
    MatrixXd D = MatrixXd::Zero(polynomial_order_, polynomial_order_);
    MatrixXd H = MatrixXd::Zero(polynomial_order_, polynomial_order_);
    VectorXd g_x = VectorXd::Zero(polynomial_order_);
    VectorXd g_y = VectorXd::Zero(polynomial_order_);

    for (int point_index = 0; point_index < estimating_odom_number_; ++point_index)
      {
        VectorXd t_t = VectorXd::Zero(polynomial_order_);
        //t_t_d is t_t after derivation_order operation
        VectorXd t_t_d = VectorXd::Zero(polynomial_order_);
        t_t[0] = 1;
        t_t_d[derivation_order_] = 1;
        for (int i = 1; i < polynomial_order_; ++i)
          {
            t_t[i] = t_t[i-1] * ((*truck_odom_time_)[point_index] - estimating_start_time_);
            if (i >= derivation_order_)
              //t_t_d[i] = t_t[i-derivation_order_];
              t_t_d[i] = t_t_d[i-1] * ((*truck_odom_time_)[point_index] + smooth_forward_time_ - estimating_start_time_);
          }
        T = T + t_t * t_t.transpose();
        D = D + t_t_d * t_t_d.transpose();
        g_x = g_x + (-2) * t_t * (*truck_odom_x_)[point_index];
        g_y = g_y + (-2) * t_t * (*truck_odom_y_)[point_index];
      }

    // Assign value for H matrix
    H = 2 * T + lambda_D_ * estimating_odom_number_ * D;

    /* Setting up QProblemB object. */

    QProblemB exampleQ_x( polynomial_order_ );
    QProblemB exampleQ_y( polynomial_order_ );

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
    exampleQ_x.init(H.data(),g_x.data(),NULL,NULL, nWSR_x,0 );
    exampleQ_x.getPrimalSolution(truck_traj_param_x_->data());

    int_t nWSR_y = 10;
    exampleQ_y.init(H.data(),g_y.data(),NULL,NULL, nWSR_y,0 );
    exampleQ_y.getPrimalSolution(truck_traj_param_y_->data());
    //printf( " ];  objVal = %e\n\n", exampleQ.getObjVal() );

    // std::cout <<"[x]: ";
    // for (int i = 0; i < polynomial_order_; ++i)
    //   std::cout << truck_traj_param_x_->data()[i] << ", ";
    // printf("\n");

  }

  void TruckTrajectoryEstimator::trajectoryVisualization()
  {
    for (int point_id = 0; point_id < estimating_odom_number_; ++point_id)
      {
        geometry_msgs::PoseStamped cur_pose;
        double delta_t = (*truck_odom_time_)[point_id] - trajectory_start_time_;
        cur_pose = truck_origin_path_->poses[point_id];
        cur_pose.pose.position.x = getPointFromPolynomial('x', delta_t);
        cur_pose.pose.position.y = getPointFromPolynomial('y', delta_t);
        truck_traj_path_->poses.push_back(cur_pose);
      }

    uav_commander_.updateUavTruckRelPos();
    uav_start_pos_ = uav_commander_.m_uav_truck_world_pos;

    int predict_number = int(visualization_predict_time_ / visualization_predict_time_unit_);
    double delta_t_offset = (*truck_odom_time_)[estimating_odom_number_-1] - trajectory_start_time_;
    geometry_msgs::PoseStamped last_pose = truck_origin_path_->poses[estimating_odom_number_-1];
    for (int point_id = 0; point_id < predict_number; ++point_id)
      {
        geometry_msgs::PoseStamped cur_pose;
        double delta_t = delta_t_offset + point_id * visualization_predict_time_unit_;
        cur_pose = last_pose;
        cur_pose.pose.position.x = getPointFromPolynomial('x', delta_t);
        cur_pose.pose.position.y = getPointFromPolynomial('y', delta_t);
        truck_traj_path_->poses.push_back(cur_pose);
        // Uav's predicted traj = offset + truck's predicted traj
        cur_pose.pose.position.x += uav_start_pos_.getX();
        cur_pose.pose.position.y += uav_start_pos_.getY();
        cur_pose.pose.position.z += uav_start_pos_.getZ();
        uav_des_traj_path_->poses.push_back(cur_pose);
      }
    pub_truck_traj_path_.publish(*truck_traj_path_);
    if (uav_state_ == 3)
      pub_uav_des_traj_path_.publish(*uav_des_traj_path_);
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
    for (int i = 0; i < polynomial_order_; ++i)
      {
        if (axis == 'x')
          result += (*truck_traj_param_x_)[i] * temp;
        else if (axis == 'y')
          result += (*truck_traj_param_y_)[i] * temp;
        temp *= var_value;
      }
    return result;
  }


  Vector3d TruckTrajectoryEstimator::nOrderPolynomial(int n, double t)
  {
    double temp = 1, delta_t = t-estimating_start_time_;
    Vector3d result(0.0, 0.0, 0.0);
    for (int i = n; i < polynomial_order_; ++i)
      {
        int factor = factorial(i, n);
        result[0] += factor * (*truck_traj_param_x_)[i] * temp;
        result[1] += factor * (*truck_traj_param_y_)[i] * temp;
        temp *= delta_t;
      }
    return result;
  }

  void TruckTrajectoryEstimator::traj_estimate_config_callback(quadrotor_trajectory::TrajectoryEstimateConfig &config, uint32_t level)
  {
    if (config.enable)
      {
        ROS_WARN("new config");
        polynomial_order_ = config.polynomial_order;
        derivation_order_ = config.derivation_order;
        lambda_D_= config.lambda_D;
        estimating_odom_number_ = config.estimating_odom_number;
        traj_generate_freq_ = config.trajectory_generate_frequency;
        smooth_forward_time_ = config.smooth_forward_time;
        visualization_predict_time_ = config.visualization_predict_time;
        uav_commander_.m_direct_pid_mode = config.direct_pid_mode;
        uav_commander_.m_traj_track_p_gain = config.traj_track_p_param;
        uav_commander_.m_traj_track_i_gain = config.traj_track_i_param;
        uav_commander_.m_traj_track_d_gain = config.traj_track_d_param;
      }
  }
}
