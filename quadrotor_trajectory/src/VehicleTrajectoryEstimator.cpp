
#include <quadrotor_trajectory/VehicleTrajectoryEstimator.h>

USING_NAMESPACE_QPOASES
namespace vehicle_trajectory_estimator
{
  void VehicleTrajectoryEstimator::onInit()
  {
    ros::NodeHandle pnh("~");
    /* ROS Node */
    pnh.param("vehicle_odom_sub_topic_name", m_vehicle_odom_sub_topic_name, std::string("/truck_odom"));
    pnh.param("vehicle_traj_param_pub_topic_name", m_vehicle_traj_param_pub_topic_name, std::string("/vehicle_traj_param"));
    pnh.param("vehicle_traj_path_pub_topic_name", m_vehicle_traj_path_pub_topic_name, std::string("/vehicle_traj_path"));
    pnh.param("vehicle_traj_polynomial_order", m_vehicle_traj_order, 8);
    pnh.param("vehicle_traj_derivation_order", m_vehicle_traj_dev_order, 4);
    pnh.param("vehicle_lambda_D", m_vehicle_lambda_D, 1.0);
    pnh.param("vehicle_estimate_odom_number", m_n_vehicle_estimate_odom, 40);
    pnh.param("vehicle_trajectory_generate_freqency", m_vehicle_traj_generate_freq, 3);
    pnh.param("vehicle_visualization_predict_time", m_vehicle_vis_predict_time, 5.0);
    pnh.param("vehicle_vis_predict_time_unit", m_vehicle_vis_predict_time_unit, 0.2);
    pnh.param("vehicle_visualization_preview_time", m_vehicle_vis_preview_time, 2.0);
    pnh.param("vehicle_smooth_forward_time", m_vehicle_smooth_forward_time, 5.0);
    pnh.param("vehicle_trajectory_deviation_threshold", m_vehicle_traj_deviation_threshold, 1.5);
    pnh.param("vehicle_max_velocity", m_vehicle_max_vel, 4.5);
    pnh.param("vehicle_max_acceleration", m_vehicle_max_acc, 1.5);
    pnh.param("vehicle_cable_height", m_vehicle_cable_height, 0.5);
    pnh.param("param_output_param", m_display_param, false);


    m_n_current_odom = 0;
    m_n_vehicle_new_odom = 0;
    m_vehicle_odom_update = false;
    m_vehicle_odom_empty_flag = true;
    m_vehicle_odom_filled_flag = false;
    m_vehicle_traj_param_x_ptr = new VectorXd(m_vehicle_traj_order);
    m_vehicle_traj_param_y_ptr = new VectorXd(m_vehicle_traj_order);

    m_sub_vehicle_odom = m_nh.subscribe<nav_msgs::Odometry>(m_vehicle_odom_sub_topic_name, 1, &VehicleTrajectoryEstimator::vehicleOdomCallback, this);

    m_pub_vehicle_traj_param = m_nh.advertise<quadrotor_trajectory::TrackParamStamped>(m_vehicle_traj_param_pub_topic_name, 1);
    m_pub_vehicle_traj_path = m_nh.advertise<nav_msgs::Path>(m_vehicle_traj_path_pub_topic_name, 1);
  }

  // 50 hz in sim
  void VehicleTrajectoryEstimator::vehicleOdomCallback(const nav_msgs::OdometryConstPtr& vehicle_odom_msg)
  {
    m_vehicle_odom = *vehicle_odom_msg;

    // Get first vehicle odom
    if (m_vehicle_odom_empty_flag)
      {
        // Init for variables in callback function
        m_vehicle_estimate_start_time = m_vehicle_odom.header.stamp.toSec();
        m_vehicle_pos_x_ptr = new VectorXd(m_n_vehicle_estimate_odom);
        m_vehicle_pos_y_ptr = new VectorXd(m_n_vehicle_estimate_odom);
        m_vehicle_odom_time_ptr = new VectorXd(m_n_vehicle_estimate_odom);
        m_vehicle_odom_empty_flag = false;
      }

    bool update_vehicle_traj_param = false;
    if (m_vehicle_odom_filled_flag)
      {
        // Erase the first element in array data
        for (int i = 0; i < m_n_vehicle_estimate_odom-1; ++i)
          {
            (*m_vehicle_odom_time_ptr)[i] = (*m_vehicle_odom_time_ptr)[i+1];
            (*m_vehicle_pos_x_ptr)[i] = (*m_vehicle_pos_x_ptr)[i+1];
            (*m_vehicle_pos_y_ptr)[i] = (*m_vehicle_pos_y_ptr)[i+1];
          }

        m_vehicle_estimate_start_time = (*m_vehicle_odom_time_ptr)[0];

        (*m_vehicle_odom_time_ptr)[m_n_vehicle_estimate_odom-1] = m_vehicle_odom.header.stamp.toSec();
        (*m_vehicle_pos_x_ptr)[m_n_vehicle_estimate_odom-1] = m_vehicle_odom.pose.pose.position.x;
        (*m_vehicle_pos_y_ptr)[m_n_vehicle_estimate_odom-1] = m_vehicle_odom.pose.pose.position.y;

        ++m_n_vehicle_new_odom;
        // Update according to fixed frequency, or special situation happens, that vehicle's trajectory estimation is wrong and needed update its trajectory.
        if (m_n_vehicle_new_odom >= m_vehicle_traj_generate_freq || isVehicleDeviateTrajectory(m_vehicle_traj_deviation_threshold, m_vehicle_odom.pose.pose.position, (*m_vehicle_odom_time_ptr)[m_n_vehicle_estimate_odom-1]))
          {
            m_n_vehicle_new_odom = 0;
            delete m_vehicle_traj_path_ptr;
            m_vehicle_traj_path_ptr = new nav_msgs::Path();
            m_vehicle_traj_path_ptr->header = m_vehicle_odom.header;
            m_vehicle_traj_start_time = m_vehicle_estimate_start_time;
            vehicleTrajectoryEstimation();
            trajectoryVisualization();
            update_vehicle_traj_param = true;
          }
      }
    // Vehicle's trajectory is firstly estimated after enough odom is got.
    else
      {
        (*m_vehicle_odom_time_ptr)[m_n_current_odom] = m_vehicle_odom.header.stamp.toSec();
        (*m_vehicle_pos_x_ptr)[m_n_current_odom] = m_vehicle_odom.pose.pose.position.x;
        (*m_vehicle_pos_y_ptr)[m_n_current_odom] = m_vehicle_odom.pose.pose.position.y;

        ++m_n_current_odom;
        if (m_n_current_odom >= m_n_vehicle_estimate_odom)
          {
            m_vehicle_odom_filled_flag = true;
            m_vehicle_traj_path_ptr = new nav_msgs::Path();
            m_vehicle_traj_path_ptr->header = m_vehicle_odom.header;
            m_vehicle_traj_start_time = m_vehicle_estimate_start_time;
            vehicleTrajectoryEstimation();
            trajectoryVisualization();
            update_vehicle_traj_param = true;
          }
      }
    if (update_vehicle_traj_param){
      //std_msgs::Float64MultiArray param_array;
      quadrotor_trajectory::TrackParamStamped param_array;
      param_array.header = m_vehicle_odom.header;
      param_array.params.layout.dim.push_back(std_msgs::MultiArrayDimension());
      param_array.params.layout.dim.push_back(std_msgs::MultiArrayDimension());
      param_array.params.layout.dim[0].label = (std::string)("x");
      param_array.params.layout.dim[0].size = m_vehicle_traj_order;
      param_array.params.layout.dim[0].stride = m_vehicle_traj_order*2 + 1;
      param_array.params.layout.dim[1].label = (std::string)("y");
      param_array.params.layout.dim[1].size = m_vehicle_traj_order;
      param_array.params.layout.dim[1].stride = m_vehicle_traj_order;

      double cur_start_time_offset = (*m_vehicle_odom_time_ptr)[m_n_vehicle_estimate_odom-1] - m_vehicle_traj_start_time;
      param_array.params.data.push_back(cur_start_time_offset);
      for (int i = 0; i < m_vehicle_traj_order; ++i){
        param_array.params.data.push_back((*m_vehicle_traj_param_x_ptr)[i]);
      }
      for (int i = 0; i < m_vehicle_traj_order; ++i){
        param_array.params.data.push_back((*m_vehicle_traj_param_y_ptr)[i]);
      }
      m_pub_vehicle_traj_param.publish(param_array);
    }
  }

  void VehicleTrajectoryEstimator::vehicleTrajectoryEstimation()
  {
    MatrixXd T = MatrixXd::Zero(m_vehicle_traj_order * 2, m_vehicle_traj_order * 2);
    MatrixXd D = MatrixXd::Zero(m_vehicle_traj_order * 2, m_vehicle_traj_order * 2);
    MatrixXd H = MatrixXd::Zero(m_vehicle_traj_order * 2, m_vehicle_traj_order * 2);
    VectorXd g = VectorXd::Zero(m_vehicle_traj_order * 2);

    for (int point_index = 0; point_index < m_n_vehicle_estimate_odom; ++point_index)
      {
        MatrixXd t_t = MatrixXd::Zero(m_vehicle_traj_order * 2, 2);
        t_t(0, 0) = 1; t_t(m_vehicle_traj_order, 1) = 1;
        for (int i = 1; i < m_vehicle_traj_order; ++i)
          {
            t_t(i, 0) = t_t(i-1, 0) * ((*m_vehicle_odom_time_ptr)[point_index] - m_vehicle_estimate_start_time);
            t_t(i + m_vehicle_traj_order, 1) = t_t(i, 0);
          }
        T = T + t_t * t_t.transpose();
        Vector2d Pi((*m_vehicle_pos_x_ptr)[point_index], (*m_vehicle_pos_y_ptr)[point_index]);
        g = g - 2 * t_t * Pi;
      }

    MatrixXd t_t_d = MatrixXd::Zero(m_vehicle_traj_order * 2, 2);
    double mul_d = 1.0, mul_factor = (*m_vehicle_odom_time_ptr)[m_n_vehicle_estimate_odom-1] - m_vehicle_estimate_start_time + m_vehicle_smooth_forward_time;
    for (int i = m_vehicle_traj_dev_order; i < m_vehicle_traj_order; ++i)
      {
        t_t_d(i, 0) = factorial(i, m_vehicle_traj_dev_order) * mul_d;
        t_t_d(i + m_vehicle_traj_order, 1) = t_t_d(i, 0);
        mul_d = mul_d * mul_factor;
      }
    D = t_t_d * t_t_d.transpose();

    for (int row_index = m_vehicle_traj_dev_order; row_index < m_vehicle_traj_order; ++row_index)
      for (int col_index = m_vehicle_traj_dev_order; col_index < m_vehicle_traj_order; ++col_index)
        {
          D(row_index, col_index) = D(row_index, col_index) * mul_factor / double(row_index+col_index-m_vehicle_traj_dev_order*2+1);
          D(row_index+m_vehicle_traj_order, col_index+m_vehicle_traj_order) = D(row_index, col_index);
        }

    // Add constraints
    // Check velocity and acceleration in future 0~2 second, and every 0.2 second
    int n_constraints = int(3.0/0.1);
    MatrixXd A = MatrixXd::Zero(n_constraints*2, m_vehicle_traj_order);
    VectorXd lb_A = VectorXd::Zero(n_constraints*2);
    VectorXd ub_A = VectorXd::Zero(n_constraints*2);
    for (int i = 0; i < n_constraints; ++i)
      {
        /*
          A(2*i, j) = pow(cur_t, j-1) * j;
          A(2*i+1, j) = pow(cur_t, j-2) * j*(j-1);
          // For efficiency, in order to avoid multiple calculation of cur_t*cur_t, use the following iterative way.
         */
        double cur_t = 0.1 * i;
        double mul = 1;
        A(2*i, 1) = 1;
        for (int j = 2; j < m_vehicle_traj_order; ++j)
          {
            A(2*i+1, j) = j * (j - 1) * mul;
            mul *= cur_t;
            A(2*i, j) = j * mul;
          }
        lb_A(2*i) = -m_vehicle_max_vel;
        lb_A(2*i+1) = -m_vehicle_max_acc;
        ub_A(2*i) = m_vehicle_max_vel;
        ub_A(2*i+1) = m_vehicle_max_acc;
      }

    // Assign value for H matrix
    H = 2 * T + m_vehicle_lambda_D * m_n_vehicle_estimate_odom * D;

    /* Setting up QProblemB object. */
    real_t *H_r = new real_t[m_vehicle_traj_order * m_vehicle_traj_order * 4];
    real_t *g_r = new real_t[m_vehicle_traj_order * 2];
    for (int i = 0; i < m_vehicle_traj_order * 2; ++i){
      int row_s = i * m_vehicle_traj_order * 2;
      for (int j = 0; j < m_vehicle_traj_order * 2; ++j){
        H_r[row_s + j] = H(i, j);
      }
    }
    for (int i = 0; i < m_vehicle_traj_order * 2; ++i){
      g_r[i] = g(i);
    }

    SQProblem exampleQ(2 * m_vehicle_traj_order,0 );

    Options options;
    //options.enableFlippingBounds = BT_FALSE;
    // options.initialStatusBounds = ST_INACTIVE;
    // options.numRefinementSteps = 1;
    // options.enableCholeskyRefactorisation = 1;
    // options.enableEqualities = BT_TRUE;
    options.printLevel = PL_LOW;
    exampleQ.setOptions( options );

    int_t nWSR_x = 30;
    //exampleQ.init(H_r, g_r, A.data(),NULL,NULL,lb_A.data(), ub_A.data(), nWSR_x,0 );
    exampleQ.init(H_r, g_r, NULL,NULL,NULL,NULL, NULL, nWSR_x,0 );
    real_t param_r[m_vehicle_traj_order * 2];
    exampleQ.getPrimalSolution(param_r);

    /* Print qp's paramater and result for vehicle trajectory estimation. */
    //printf( " ];  objVal = %e\n\n", exampleQ.getObjVal() );


    for (int i = 0; i < m_vehicle_traj_order; ++i){
      (*m_vehicle_traj_param_x_ptr)[i] = param_r[i];
      (*m_vehicle_traj_param_y_ptr)[i] = param_r[i + m_vehicle_traj_order];
    }

    if (m_display_param){
      std::cout << "\nVehicle trajectory estimation:\n";
      std::cout <<"[x]: ";
      for (int i = 0; i < m_vehicle_traj_order; ++i){
        std::cout << m_vehicle_traj_param_x_ptr->data()[i] << ", ";
      }
      printf("\n");
      std::cout <<"[y]: ";
      for (int i = 0; i < m_vehicle_traj_order; ++i){
        std::cout << m_vehicle_traj_param_y_ptr->data()[i] << ", ";
      }
      printf("\nVehicle trajectory finished.\n\n");
    }

    // std::cout << "Start:\n Matrix H: \n";
    // for (int i = 0; i < m_vehicle_traj_order; ++i){
    //   std::cout << "l" << i << ": ";
    //   for (int j = 0; j < m_vehicle_traj_order; ++j){
    //     std::cout << H(i,j) << ' ';
    //   }
    //   std::cout << "\n";
    // }

    // std::cout << "\nMatrix A: \n";
    // for (int i = 0; i < n_constraints*2; ++i){
    //   std::cout << "l" << i << ": ";
    //   for (int j = 0; j < m_vehicle_traj_order; ++j){
    //     std::cout << A(i,j) << ' ';
    //   }
    //   std::cout << "\n";
    // }
    // std::cout << "\n g_x: \n";
    // for (int i = 0; i < m_vehicle_traj_order; ++i){
    //   std::cout << g_x(i) << ' ';
    // }
    // std::cout << "\n g_y: \n";
    // for (int i = 0; i < m_vehicle_traj_order; ++i){
    //   std::cout << g_y(i) << ' ';
    // }

    // std::cout << "\nLower bound A: \n";
    // for (int i = 0; i < n_constraints*2; ++i){
    //   std::cout << lb_A(i) << ' ';
    // }
    // std::cout << "\nUpper bound A: \n";
    // for (int i = 0; i < n_constraints*2; ++i){
    //   std::cout << ub_A(i) << ' ';
    // }
    // std::cout << "\n\n";

  }

  void VehicleTrajectoryEstimator::trajectoryVisualization()
  {
    int vehicle_predict_number = int(m_vehicle_vis_predict_time / m_vehicle_vis_predict_time_unit);
    int vehicle_preview_number = int(m_vehicle_vis_preview_time / m_vehicle_vis_predict_time_unit);
    double delta_t_offset = (*m_vehicle_odom_time_ptr)[m_n_vehicle_estimate_odom-1] - m_vehicle_traj_start_time;
    // Preview time to examine its result for old traj, predict time to estimate the future traj
    for (int point_id = -vehicle_preview_number; point_id < vehicle_predict_number; ++point_id)
      {
        geometry_msgs::PoseStamped cur_pose;
        double delta_t = delta_t_offset + point_id * m_vehicle_vis_predict_time_unit;
        cur_pose.header = m_vehicle_odom.header;
        cur_pose.pose.position.x = getPointFromVehicleTrajectory('x', delta_t);
        cur_pose.pose.position.y = getPointFromVehicleTrajectory('y', delta_t);
        m_vehicle_traj_path_ptr->poses.push_back(cur_pose);
      }
    m_pub_vehicle_traj_path.publish(*m_vehicle_traj_path_ptr);
  }

  int VehicleTrajectoryEstimator::factorial(int n, int order)
  {
    int result = 1;
    for (int i = 0; i < order; ++i)
      result *= (n - i);
    return result;
  }

  double VehicleTrajectoryEstimator::getPointFromVehicleTrajectory(char axis, double var_value)
  {
    double result = 0, temp = 1;
    for (int i = 0; i < m_vehicle_traj_order; ++i)
      {
        if (axis == 'x')
          result += (*m_vehicle_traj_param_x_ptr)[i] * temp;
        else if (axis == 'y')
          result += (*m_vehicle_traj_param_y_ptr)[i] * temp;
        temp *= var_value;
      }
    return result;
  }


  Vector3d VehicleTrajectoryEstimator::nOrderVehicleTrajectory(int n, double t)
  {
    double temp = 1, delta_t;
    delta_t = t-m_vehicle_estimate_start_time;
    Vector3d result(0.0, 0.0, 0.0);
    for (int i = n; i < m_vehicle_traj_order; ++i)
      {
        int factor = factorial(i, n);
        result[0] += factor * (*m_vehicle_traj_param_x_ptr)[i] * temp;
        result[1] += factor * (*m_vehicle_traj_param_y_ptr)[i] * temp;
        temp *= delta_t;
      }
    return result;
  }


  double VehicleTrajectoryEstimator::realTimeCvtToVehicleTrajectoryTime(double t, Vector3d vehicle_vel)
  {
    double delta_t = t - m_vehicle_estimate_start_time;
    double segment_time_unit = 0.1;
    int segment_cnt = 0;
    double accumulation_traj = 0, sum_traj = delta_t * (sqrt(pow(vehicle_vel.x(), 2)
                                                             + (pow(vehicle_vel.y(), 2))));
    Vector3d next_pos, cur_pos;
    cur_pos = nOrderVehicleTrajectory(0, m_vehicle_estimate_start_time);
    while (1){
      if (sum_traj <= accumulation_traj)
        break;
      ++segment_cnt;
      next_pos = nOrderVehicleTrajectory(0, m_vehicle_estimate_start_time + segment_time_unit * segment_cnt);
      accumulation_traj += sqrt(pow(next_pos.x()-cur_pos.x(), 2) + pow(next_pos.y()-cur_pos.y(), 2));
      cur_pos = next_pos;
    }
    return m_vehicle_estimate_start_time + segment_time_unit * segment_cnt;
  }

  bool VehicleTrajectoryEstimator::isVehicleDeviateTrajectory(double threshold, geometry_msgs::Point vehicle_pos, double current_time)
  {
    double delta_t = current_time-m_vehicle_traj_start_time;
    double distance = pow(getPointFromVehicleTrajectory('x', delta_t)-vehicle_pos.x, 2) + pow(getPointFromVehicleTrajectory('y', delta_t)-vehicle_pos.y, 2);
    if (distance > pow(threshold, 2))
      {
        if (m_display_param){
          ROS_WARN("Vehicle is deviate from predict trajectoy.");
          std::cout << "Deviation is : " << sqrt(distance) << ", threshold is : " << threshold << "\n";
        }
        return true;
      }
    else
      return false;
  }

}
