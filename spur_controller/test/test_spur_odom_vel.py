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

import requests
from requests import ConnectionError
import unittest

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest


_PKG = 'spur'


class TestSpurOdomVel(unittest.TestCase):
    '''    '''

    @classmethod
    def setUpClass(cls):
        ''' '''        
        self.pub_vel = rospy.Publisher('/spur/cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber("odom", Odometry, _cb_odom)
        self._odom_topic = None

    @classmethod
    def tearDownClass(cls):
        True

    def _cb_odom(self, odom_topic):
        self._odom_topic = odom_topic        
        
    def test_odom_equals_sumofvel(self):
        ''' Test if odom.position is equal to summation of cmd velocity '''
        r = rospy.Rate(1)  # hz
        period = 30
        x_vel = 0.1  # m/s
        # This should move the robot 3cm ahead, so does odom count.
        for i in period:
            cmd_vel = Twist()
            cmd_vel.linear.x = x_vel
            self.pub_vel.publish(cmd_vel)
            r.sleep()
        
        # Get the odom and compare
        assertEqual(self._odom_topic.pose.pose.position.x, period * x_vel)


if __name__ == '__main__':
    rostest.rosrun(_PKG, 'test_spur_odom_vel', TestSpurOdomVel)
