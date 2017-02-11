#include <quadrotor_trajectory/VehicleTrajectoryEstimator.h>

using namespace vehicle_trajectory_estimator;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vechile_estimator");
  VehicleTrajectoryEstimator vehicle_estimator;
  vehicle_estimator.onInit();
  ros::spin();
  return 0;
}
