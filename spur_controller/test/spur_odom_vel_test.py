#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac I.Y. Saito

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest

_PKG = 'spur'

class SpurOdomVelTest():
    '''Intended to be runnable both standalone and from rostest'''

    def _cb_odom(self, odom_topic):
        self.odom_topic = odom_topic        

    def __init__(self):
        rospy.init_node('spur_odom_vel_test')
        self._pub_vel = rospy.Publisher('/spur/cmd_vel', Twist, queue_size=10)
        self._sub_odom = rospy.Subscriber("/odom", Odometry, self._cb_odom)
        self.odom_topic = None
        
    def odom_equals_sumofvel_test(self):
        ''' Test if odom.position is equal to summation of cmd velocity '''
        r = rospy.Rate(0.5)  # hz
        period = 12  # second
        x_vel = 0.1  # m/s
        y_vel = 0.1
        yaw_vel = 1.5708  # rad 
        for i in range(period):
            cmd_vel = Twist()
            cmd_vel.linear.x = x_vel
            if i == 3 or i == 6 or i == 9:
                cmd_vel.angular.z = yaw_vel
                #cmd_vel.linear.y = y_vel
                #x_vel = -x_vel
                            
            self._pub_vel.publish(cmd_vel)
            r.sleep()
        # The loop above should move the robot (x_vel * period) meter ahead, so does odom count.

        # Get the odom and compare
        odom_x = self.odom_topic.pose.pose.position.x
        sum_vel_x = period * x_vel
        if odom_x == sum_vel_x:  # TODO: needs to be approximated
            rospy.loginfo('Odom is equal to the sum')
            return True
        else:
            rospy.logerr('Odom={}, sum_vel_x={}'.format(odom_x, sum_vel_x))
            return False

if __name__ == '__main__':
    SpurOdomVelTest().odom_equals_sumofvel_test()
