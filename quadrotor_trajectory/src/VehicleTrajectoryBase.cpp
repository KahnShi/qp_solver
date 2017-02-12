
#include <quadrotor_trajectory/VehicleTrajectoryBase.h>

//USING_NAMESPACE_QPOASES
namespace vehicle_trajectory_base
{
  void VehicleTrajectoryBase::onInit(int order, std::vector<double> &data)
  {
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
}
