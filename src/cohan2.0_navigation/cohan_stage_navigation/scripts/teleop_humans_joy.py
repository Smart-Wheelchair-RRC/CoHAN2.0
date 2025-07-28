#!/usr/bin/env python3
"""
Software License Agreement (MIT License)

Copyright (c) 2020-2025 LAAS-CNRS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Author: Phani Teja Singamaneni
"""

# Brief: Node for controlling human avatars through joy stick
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopHumans(object):
  def __init__(self, h_id):
    self.h_id = h_id
    self.angular_ = 3
    self.linear_x = 1
    self.linear_y = 0
    rospy.init_node("teleop_humans_joy")
    rospy.Subscriber("joy", Joy, self.JoyCB)
    name = '/human'+str(self.h_id)+'/cmd_vel'
    self.vel_pub = rospy.Publisher(name, Twist, queue_size = 1)

    rospy.spin()

  def JoyCB(self, msg):
    velocity = Twist()
    velocity.linear.x = 2.0*msg.axes[self.linear_x]
    velocity.linear.y = 2.0*msg.axes[self.linear_y]
    velocity.angular.z = 2.0*msg.axes[self.angular_]
    self.vel_pub.publish(velocity)


if __name__ == '__main__':
  teleop_joy = TeleopHumans(1)
