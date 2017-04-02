
#include <quadrotor_trajectory/VehicleTrajectoryBase.h>

//USING_NAMESPACE_QPOASES
namespace vehicle_trajectory_base
{
  VehicleTrajectoryBase::VehicleTrajectoryBase()
  {
    m_init_flag = false;
  }

  void VehicleTrajectoryBase::onInit(int order, std::vector<double> &data)
  {
    m_init_flag = true;
    m_vehicle_traj_order = order;
    m_vehicle_traj_param_x_ptr = new VectorXd(m_vehicle_traj_order);
    m_vehicle_traj_param_y_ptr = new VectorXd(m_vehicle_traj_order);
    m_vehicle_traj_start_time = data[0];
    for (int i = 0; i < m_vehicle_traj_order; ++i){
      (*m_vehicle_traj_param_x_ptr)[i] = data[i+1];
      (*m_vehicle_traj_param_y_ptr)[i] = data[i+1+m_vehicle_traj_order];
    }
  }


  int VehicleTrajectoryBase::factorial(int n, int order)
  {
    int result = 1;
    for (int i = 0; i < order; ++i)
      result *= (n - i);
    return result;
  }

  Vector3d VehicleTrajectoryBase::nOrderVehicleTrajectory(int n, double t)
  {
    if (!m_init_flag){
      ROS_WARN("VehicleTrajectoryBase class is not initialized.");
      return Vector3d(0.0, 0.0, 0.0);
    }

    double temp = 1, delta_t;
    delta_t = t+m_vehicle_traj_start_time;
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

  bool VehicleTrajectoryBase::isVehicleDeviateTrajectory(double threshold, geometry_msgs::Point vehicle_pos, double t)
  {
    if (!m_init_flag){
      ROS_WARN("VehicleTrajectoryBase class is not initialized.");
      return true;
    }

    Vector3d traj_pos = nOrderVehicleTrajectory(0, t);
    double distance = pow(traj_pos[0] - vehicle_pos.x, 2) + pow(traj_pos[1] - vehicle_pos.y, 2);
    if (distance > pow(threshold, 2))
      {
        ROS_WARN("Vehicle is deviate from predict trajectoy.");
        std::cout << "Deviation is : " << sqrt(distance) << ", threshold is : " << threshold << "\n";
        return true;
      }
    else
      return false;
  }

  void VehicleTrajectoryBase::printAll()
  {
    if (!m_init_flag){
      ROS_WARN("VehicleTrajectoryBase class is not initialized.");
      return;
    }
    std::cout << "Params number: " << m_vehicle_traj_order << "\n";
    std::cout << "Param x: ";
    for (int i = 0; i < m_vehicle_traj_order; ++i){
      std::cout << (*m_vehicle_traj_param_x_ptr)[i] << ", ";
    }
    std::cout << "\nParam y: ";
    for (int i = 0; i < m_vehicle_traj_order; ++i){
      std::cout << (*m_vehicle_traj_param_y_ptr)[i] << ", ";
    }
    std::cout << "\n\n";
  }
}
