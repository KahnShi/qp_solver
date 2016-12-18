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
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64, Empty, Bool, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as transformation
import math

class simulate_position:

    def init(self):
        rospy.init_node('simulate_position', anonymous=True)

        ## Subscriber
        ## 100 hz
        #self.__subscriber_uav_odom = rospy.Subscriber("/ground_truth/state", Odometry, self.__uav_odom_callback)
        ## 50 hz
        #self.__subscriber_truck_odom = rospy.Subscriber("/truck/ground_truth/odom", Odometry, self.__truck_odom_callback)

        ## Publisher
        self.__publisher_truck_pos = rospy.Publisher("/simulating_truck_pos", PoseStamped, queue_size = 10)
        self.__publisher_truck_odom = rospy.Publisher("/simulating_truck_odom", Odometry, queue_size = 10)
        time.sleep(3)
        self.__publish_pos()

    def __publish_pos(self):
        header = Header()
        header.seq = 0
        header.frame_id = "world"
        header.stamp = rospy.Time.now()

        points_num = 20
        period_time = 2.0
        circle_radius = 20.0
        traverse_angle = 5.0 * period_time / circle_radius
        origin_point = [0, 0]
        start_time = 0.0
        time_sum = 2.0
        for i in range(0, points_num):
            cur_point = [circle_radius * math.cos(traverse_angle * i / points_num) + origin_point[0], circle_radius * math.sin(traverse_angle * i / points_num) + origin_point[1]]
            cur_time = start_time + i * time_sum / points_num
            cur_pose = PoseStamped()
            cur_pose.pose.position.x = cur_point[0]
            cur_pose.pose.position.y = cur_point[1]
            cur_pose.pose.position.z = 0.0
            header.seq += 1
            header.stamp = rospy.Time.now()
            cur_pose.header = header
            cur_odom = Odometry()
            cur_odom.header = header
            cur_odom.pose.pose = cur_pose.pose
            self.__publisher_truck_pos.publish(cur_pose)
            self.__publisher_truck_odom.publish(cur_odom)
            time.sleep(0.1)
        print "Truck simulating pos publish finished."

if __name__ == '__main__':
    try:
        mySim = simulate_position()
        mySim.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
