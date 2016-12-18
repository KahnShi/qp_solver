#include <quadrotor_trajectory/TruckTrajectoryEstimator.h>

using namespace truck_trajectory_estimator;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "truck_estimator");
  //ros::NodeHandle nh("truck_estimator");
  //ros::NodeHandle pnh("~");
  TruckTrajectoryEstimator truck_estimator;
  truck_estimator.onInit();
  ros::spin();
  return 0;
}
