
#include <quadrotor_trajectory/TruckTrajectoryEstimator.h>


namespace truck_trajectory_estimator
{
  void TruckTrajectoryEstimator::onInit()
  {
    ros::NodeHandle pnh_("~");
    /* ROS Node */
    pnh_.param("truck_odom_sub_topic_name", truck_odom_sub_topic_name_, std::string("/simulating_truck_odom"));
    pnh_.param("polynomial_order", polynomial_order_, 6);
    pnh_.param("solver_mode", solver_mode_, 0);
    pnh_.param("lambda_D", lambda_D_, 0.01);
    pnh_.param("estimating_pos_number", estimating_pos_number_, 20);

    current_pos_number_ = 0;
    truck_odom_update_ = false;
    first_truck_odom_flag_ = true;
    estimating_truck_odom_flag_ = true;
    markerInit(truck_marker_);

    sub_truck_odom_ = nh_.subscribe<nav_msgs::Odometry>(truck_odom_sub_topic_name_, 1, &TruckTrajectoryEstimator::truckOdomCallback, this);

    pub_truck_origin_path_ = nh_.advertise<nav_msgs::Path>("/truck_path", 1);
    pub_truck_traj_path_ = nh_.advertise<nav_msgs::Path>("/trajectory_path", 1);
    pub_truck_origin_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/truck_markers", 1);
    pub_truck_traj_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);

  }


  void TruckTrajectoryEstimator::truckOdomCallback(const nav_msgs::OdometryConstPtr& truck_odom_msg)
  {
    nav_msgs::Odometry truck_odom = *truck_odom_msg;
    //std::cout << "received \n";
    if (estimating_truck_odom_flag_)
      {
        estimating_truck_odom_flag_ = false;
        estimating_start_time_ = truck_odom.header.stamp.toSec();
        truck_pos_x_ = new VectorXd(estimating_pos_number_);
        truck_pos_y_ = new VectorXd(estimating_pos_number_);
        truck_pos_time_ = new VectorXd(estimating_pos_number_);
        truck_traj_path_ = new nav_msgs::Path();
        truck_traj_path_->header = truck_odom.header;
        if (first_truck_odom_flag_)
          {
            first_truck_odom_flag_ = false;
            truck_origin_path_ = new nav_msgs::Path();
            truck_origin_path_->header = truck_odom.header;
            truck_origin_markers_ = new visualization_msgs::MarkerArray();
          }
      }
    (*truck_pos_time_)[current_pos_number_] = truck_odom.header.stamp.toSec() - estimating_start_time_;
    (*truck_pos_x_)[current_pos_number_] = truck_odom.pose.pose.position.x;
    (*truck_pos_y_)[current_pos_number_] = truck_odom.pose.pose.position.y;

    geometry_msgs::PoseStamped cur_pose_stamped;
    cur_pose_stamped.header = truck_odom.header;
    cur_pose_stamped.pose = truck_odom.pose.pose;
    truck_origin_path_->poses.push_back(cur_pose_stamped);
    truck_marker_.pose = truck_odom.pose.pose;
    truck_marker_.id += 1;
    truck_marker_.header = truck_odom.header;
    truck_origin_markers_->markers.push_back(truck_marker_);
    pub_truck_origin_path_.publish(*truck_origin_path_);
    pub_truck_origin_markers_.publish(*truck_origin_markers_);

    ++current_pos_number_;

    if (current_pos_number_ >= estimating_pos_number_)
      {
        estimating_truck_odom_flag_ = true;
        current_pos_number_ = 0;
        //estimating

        //~ value
        delete truck_pos_x_;
        delete truck_pos_y_;
        delete truck_pos_time_;
        delete truck_traj_path_;
      }
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
    if (color == 'r')
      marker.color.r = 1;
    else if (color == 'g')
      marker.color.g = 1;
    else if (color == 'b')
      marker.color.b = 1;
  }
}
