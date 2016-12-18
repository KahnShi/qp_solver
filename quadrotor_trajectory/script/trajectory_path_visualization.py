#!/usr/bin/env python

# Copyright (c) 2016, JSK(University of Tokyo)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Open Source Robotics Foundation, Inc.
#       nor the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior
#       written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Fan Shi, Moju Zhao
# Maintainer: Fan Shi <shifan@jsk.imi.i.u-tokyo.ac.jp> 

import time
import sys
import math
import tf
import numpy as np
from std_msgs.msg import Float64, Empty, Bool, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations as transformation
import math
from visualization_msgs.msg import Marker, MarkerArray
import copy

class trajectory_visualization:

    def init(self):
        rospy.init_node('trajectory_path_visualization', anonymous=True)

        self.__uav_odom_update = False
        self.__first_uav_odom_flag = True
        self.__truck_path = Path()
        self.__trajectory_path = Path()
        self.__truck_marker = Marker()
        self.__truck_markers = MarkerArray()
        self.__trajectory_marker = Marker()
        self.__trajectory_markers = MarkerArray()
        self.__truck_marker = self.__marker_init(self.__truck_marker, 'r')
        self.__trajectory_marker = self.__marker_init(self.__trajectory_marker, 'g')
        ## Subscriber
        ## 100 hz
        #self.__subscriber_uav_odom = rospy.Subscriber("/ground_truth/state", Odometry, self.__uav_odom_callback)
        ## 50 hz
        #self.__subscriber_truck_odom = rospy.Subscriber("/truck/ground_truth/odom", Odometry, self.__truck_odom_callback)
        self.__subscriber_truck_pose = rospy.Subscriber("/simulating_truck_pos", PoseStamped, self.__truck_pose_callback)

        ## Publisher
        self.__publisher_truck_path = rospy.Publisher("/truck_path", Path, queue_size = 10)
        self.__publisher_trajectory_path = rospy.Publisher("/trajectory_path", Path, queue_size = 10)
        self.__publisher_truck_marker = rospy.Publisher('/truck_marker', Marker, queue_size=10)
        self.__publisher_truck_marker_array = rospy.Publisher('/truck_markers', MarkerArray, queue_size=10)

        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            if not self.__first_uav_odom_flag:
                self.__publisher_truck_path.publish(self.__truck_path)
                self.__publisher_truck_marker_array.publish(self.__truck_markers)
                self.__publisher_trajectory_path.publish(self.__trajectory_path)
            rate.sleep()


    def __truck_pose_callback(self, msg):
        if self.__first_uav_odom_flag:
            self.__truck_path.header = msg.header
            self.__trajectory_path.header = msg.header
            self.__first_uav_odom_flag = False

        self.__truck_path.poses.append(msg)
        self.__truck_marker.pose = msg.pose
        self.__truck_marker.id += 1
        self.__publisher_truck_marker.publish(self.__truck_marker)
        self.__truck_markers.markers.append(self.__truck_marker)

        trajectory_pose_stamped = PoseStamped()
        trajectory_pose_stamped.header = msg.header
        current_time = 0.1 * self.__truck_marker.id
        trajectory_pose_stamped.pose = self.__trajectory_generation(current_time)
        self.__trajectory_path.poses.append(trajectory_pose_stamped)

    def __trajectory_generation(self, t):
        polynomial_order = 6
        param_x = [ 4.250094, -2.005878, -0.019644, 0.020301, 0.057058, -0.031985, 0.004601]
        param_y = [ 0.507430, -0.012978, -0.000019, 0.000164, -0.000284, 0.000118, -0.000015]
        result = [0, 0]
        for i in range(0, polynomial_order+1):
            order_value = pow(t, i)
            result[0] += param_x[i] * order_value
            result[1] += param_y[i] * order_value
        result_pos = Pose()
        result_pos.position.x = result[0]
        result_pos.position.y = result[1]
        result_pos.position.z = 0
        print result
        return result_pos

    def __marker_init(self, marker, color = 'r'):
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 0.0
        marker.color.b = 0
        if color == 'r':
            marker.color.r = 1
        elif color == 'g':
            marker.color.g = 1
        elif color == 'b':
            marker.color.b = 1
        return marker

    # Callback Func
    # def __uav_odom_callback(self, msg):
    #     self.__uav_odom_update = True
    #     self.__current_uav_msg = msg
    #     self.__uav_pos_world[0] = msg.pose.pose.position.x
    #     self.__uav_pos_world[1] = msg.pose.pose.position.y
    #     self.__uav_pos_world[2] = msg.pose.pose.position.z

    #     self.__uav_vel_world[0] = msg.twist.twist.linear.x
    #     self.__uav_vel_world[1] = msg.twist.twist.linear.y
    #     self.__uav_vel_world[2] = 0

    #     self.__uav_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    # def __truck_odom_callback(self, msg):
    #     self.__truck_pos_world[0] = msg.pose.pose.position.x
    #     self.__truck_pos_world[1] = msg.pose.pose.position.y
    #     self.__truck_pos_world[2] = msg.pose.pose.position.z

    #     self.__truck_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #     (roll,pitch,yaw) = transformation.euler_from_quaternion(self.__truck_q)

    #     self.__truck_vel_world[0] = 5 * math.cos(yaw)
    #     self.__truck_vel_world[1] = 5 * math.sin(yaw)
    #     self.__truck_vel_world[2] = 0

    #     if self.__uav_odom_update:
    #         truck_pos_ref_uav = Odometry()
    #         truck_pos_ref_uav.pose.pose.position.z = self.__uav_pos_world[2] - 1.0
    #         truck_landing_region_offset = [-0.35, 0.35]

    #         (roll,pitch,yaw) = transformation.euler_from_quaternion(self.__uav_q)
    #         R = transformation.euler_matrix(0, 0, yaw)
    #         inv_R = transformation.inverse_matrix(R)
    #         vec_pos_truck_world = [[self.__truck_pos_world[0]-self.__uav_pos_world[0]+truck_landing_region_offset[0]], [self.__truck_pos_world[1]-self.__uav_pos_world[1]+truck_landing_region_offset[1]], [0], [0]]
    #         vec_pos_truck_ref_uav = inv_R.dot(vec_pos_truck_world)
    #         truck_pos_ref_uav.pose.pose.position.x = vec_pos_truck_ref_uav[0]
    #         truck_pos_ref_uav.pose.pose.position.y = vec_pos_truck_ref_uav[1]

    #         vec_vel_truck_world = [[self.__truck_vel_world[0]], [self.__truck_vel_world[1]], [0], [0]]
    #         vec_vel_truck_ref_uav = inv_R.dot(vec_vel_truck_world)
    #         truck_pos_ref_uav.twist.twist.linear.x = vec_vel_truck_ref_uav[0]
    #         truck_pos_ref_uav.twist.twist.linear.y = vec_vel_truck_ref_uav[1]
    #         truck_pos_ref_uav.twist.twist.linear.z = 0

    #         self.__publisher_object_pos.publish(truck_pos_ref_uav)

if __name__ == '__main__':
    try:
        myVis = trajectory_visualization()
        myVis.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
