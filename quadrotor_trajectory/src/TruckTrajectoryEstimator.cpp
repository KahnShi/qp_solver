
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
    pnh_.param("solver_mode", solver_mode_, 0);
    pnh_.param("lambda_D", lambda_D_, 0.01);
    pnh_.param("estimating_odom_number", estimating_odom_number_, 20);
    pnh_.param("trajectory_generate_freqency", traj_generate_freq_, 10);
    pnh_.param("smooth_forward_time", smooth_forward_time_, 1.0);

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

    sub_truck_odom_ = nh_.subscribe<nav_msgs::Odometry>(truck_odom_sub_topic_name_, 1, &TruckTrajectoryEstimator::truckOdomCallback, this);

    pub_truck_origin_path_ = nh_.advertise<nav_msgs::Path>("/truck_path", 1);
    pub_truck_traj_path_ = nh_.advertise<nav_msgs::Path>("/trajectory_path", 1);
    pub_truck_origin_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/truck_markers", 1);
    pub_truck_traj_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);

  }


  void TruckTrajectoryEstimator::truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg)
  {
    nav_msgs::Odometry truck_odom = *truck_odom_msg;
    geometry_msgs::PoseStamped cur_truck_pose_stamped;
    cur_truck_pose_stamped.header = truck_odom.header;
    cur_truck_pose_stamped.pose = truck_odom.pose.pose;
    truck_marker_.pose = truck_odom.pose.pose;
    truck_marker_.id += 1;
    truck_marker_.header = truck_odom.header;

    //std::cout << "received \n";
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
            trajectory_start_time_ = estimating_start_time_;
            polynomial_estimation();
            trajectory_visualization_same_odompoints(0);
          }
        else
          {
            trajectory_visualization_same_odompoints(1);
          }
      }
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
            polynomial_estimation();
            trajectory_visualization_same_odompoints(0);
          }
      }
    pub_truck_origin_path_.publish(*truck_origin_path_);
    pub_truck_origin_markers_.publish(*truck_origin_markers_);
  }

  void TruckTrajectoryEstimator::polynomial_estimation()
  {
    MatrixXd T = MatrixXd::Zero(polynomial_order_, polynomial_order_);
    MatrixXd D = MatrixXd::Zero(polynomial_order_, polynomial_order_);
    MatrixXd H = MatrixXd::Zero(polynomial_order_, polynomial_order_);
    VectorXd g_x = VectorXd::Zero(polynomial_order_);
    VectorXd g_y = VectorXd::Zero(polynomial_order_);

    // for (int point_index = 0; point_index < estimating_odom_number_; ++point_index)
    //   {
    //     for (int row_id = 0; row_id < polynomial_order_; ++row_id)
    //       {
    //         double t_add = pow((*truck_odom_time_)[point_index], row_id);
    //         // Assign value for g vector
    //         g_x[row_id] += (-2*(*truck_odom_x_)[point_index] * t_add);
    //         g_y[row_id] += (-2*(*truck_odom_y_)[point_index] * t_add);

    //         for (int col_id = 0; col_id < polynomial_order_; ++col_id)
    //           {
    //             T(row_id, col_id) += pow((*truck_odom_time_)[point_index], (row_id+col_id));
    //             if (row_id >= derivation_order_ and col_id >= derivation_order_)
    //               D(row_id, col_id) += pow((*truck_odom_time_)[point_index]+1.0, (row_id+col_id-2*derivation_order_+1)) * pow(factorial(row_id,derivation_order_), 2) / (row_id+col_id-2*derivation_order_+1);
    //           }
    //       }
    //   }

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

    QProblemB exampleQ( polynomial_order_ );
    SQProblem exampleSQ( polynomial_order_, 0);

    Options options;
    //options.enableFlippingBounds = BT_FALSE;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;

    // Bakui's
    options.enableEqualities = BT_TRUE;
    options.printLevel = PL_LOW;
    if (solver_mode_ == 0)
      exampleQ.setOptions( options );
    else if (solver_mode_ == 1)
      exampleSQ.setOptions( options );


    /* Solve first QP. */
    int_t nWSR = 10;
    if (solver_mode_ == 0)
      //exampleQ.init( (const real_t*)(H.data()),(const real_t*)(g_x.data()),NULL,NULL, nWSR,0 );
      exampleQ.init(H.data(),g_x.data(),NULL,NULL, nWSR,0 );
    else if (solver_mode_ == 1)
      //exampleSQ.init( (const real_t*)(H.data()),(const real_t*)(g_x.data()),NULL,NULL,NULL,NULL,NULL,nWSR,0 );
      exampleSQ.init(H.data(),g_x.data(),NULL,NULL,NULL,NULL,NULL,nWSR,0 );

    /* Get and print solution of first QP. */
    if (solver_mode_ == 0)
      {
        exampleQ.getPrimalSolution(truck_traj_param_x_->data());
        //printf( " ];  objVal = %e\n\n", exampleQ.getObjVal() );
      }
    else if (solver_mode_ == 1)
      {
        exampleSQ.getPrimalSolution(truck_traj_param_x_->data());
        //printf( " ];  objVal = %e\n\n", exampleSQ.getObjVal() );
      }
    // printf("\nxOpt = [ ");
    std::cout <<"[x]: ";
    for (int i = 0; i < polynomial_order_; ++i)
      std::cout << truck_traj_param_x_->data()[i] << ", ";
    printf("\n");

    /* Solve second QP. */
    nWSR = 10;

    if (solver_mode_ == 0)
      //exampleQ.hotstart( (const real_t*)(g_y.data()),NULL,NULL, nWSR,0 );
      exampleQ.hotstart(g_y.data(),NULL,NULL, nWSR,0 );
    else if (solver_mode_ == 1)
      //exampleSQ.hotstart( (const real_t*)(H.data()),(const real_t*)(g_y.data()),NULL,NULL,NULL,NULL,NULL,nWSR,0 );
      exampleSQ.hotstart(H.data(),g_y.data(),NULL,NULL,NULL,NULL,NULL,nWSR,0 );

    //printf( "\nnWSR = %d\n\n", nWSR );

    /* Get and print solution of second QP. */
    if (solver_mode_ == 0)
      {
        exampleQ.getPrimalSolution(truck_traj_param_y_->data());
        //printf( " ];  objVal = %e\n\n", exampleQ.getObjVal() );
      }
    else if (solver_mode_ == 1)
      {
        exampleSQ.getPrimalSolution(truck_traj_param_y_->data());
        //printf( " ];  objVal = %e\n\n", exampleSQ.getObjVal() );
      }

    std::cout <<"[y]: ";
    for (int i = 0; i < polynomial_order_; ++i)
      std::cout << truck_traj_param_y_->data()[i] << ", ";
    printf("\n");


  }

  void TruckTrajectoryEstimator::trajectory_visualization()
  {
    // Add 3 points between 2 origin data
    int visualization_dense = 3, interpolation_number;
    double time_gap;
    std_msgs::Header start_header;
    if (new_traj_points_number_ >= traj_generate_freq_)
      {
        time_gap = (*truck_odom_time_)[estimating_odom_number_-1] - (*truck_odom_time_)[estimating_odom_number_-2] / visualization_dense;
        interpolation_number = visualization_dense;
        start_header = truck_origin_path_->poses[estimating_odom_number_-1].header;
      }
    else
      {
        time_gap = (*truck_odom_time_)[estimating_odom_number_-1] - (*truck_odom_time_)[0] / (visualization_dense*(estimating_odom_number_-1));
        interpolation_number = visualization_dense*(estimating_odom_number_-1);
        start_header = truck_origin_path_->poses[0].header;
      }
    for (int i = 0; i < visualization_dense; ++i)
      {
        geometry_msgs::PoseStamped cur_pose;
        // todo: set header time for every posestamped
        // cur_pose = truck_origin_path_->poses[point_id];
        cur_pose.pose.position.x = get_point_from_polynomial('x', time_gap*i);
        cur_pose.pose.position.y = get_point_from_polynomial('y', time_gap*i);
        truck_traj_path_->poses.push_back(cur_pose);
      }
    pub_truck_traj_path_.publish(*truck_traj_path_);
  }

  // trajectory visualization based on same odom points
  // mode 0: push_back all the poses into traj_path
  // mode 1: add new pose to traj_path
  void TruckTrajectoryEstimator::trajectory_visualization_same_odompoints(int mode)
  {
    if (mode == 0)
      {
        for (int point_id = 0; point_id < estimating_odom_number_; ++point_id)
          {
            geometry_msgs::PoseStamped cur_pose;
            double delta_t = (*truck_odom_time_)[point_id] - trajectory_start_time_;
            cur_pose = truck_origin_path_->poses[point_id];
            cur_pose.pose.position.x = get_point_from_polynomial('x', delta_t);
            cur_pose.pose.position.y = get_point_from_polynomial('y', delta_t);
            truck_traj_path_->poses.push_back(cur_pose);
          }
      }
    else
      {
        geometry_msgs::PoseStamped cur_pose;
        double delta_t = (*truck_odom_time_)[estimating_odom_number_-1] - trajectory_start_time_;
        cur_pose = truck_origin_path_->poses[estimating_odom_number_-1];
        cur_pose.pose.position.x = get_point_from_polynomial('x', delta_t);
        cur_pose.pose.position.y = get_point_from_polynomial('y', delta_t);
        truck_traj_path_->poses.push_back(cur_pose);
      }
    pub_truck_traj_path_.publish(*truck_traj_path_);
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

  double TruckTrajectoryEstimator::get_point_from_polynomial(char axis, double var_value)
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
      }
  }
}
