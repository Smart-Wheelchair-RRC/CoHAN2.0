#!/usr/bin/env python3

"""
Software License Agreement (MIT License)

Copyright (c) 2020–2025 LAAS-CNRS

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

"""
@file agent_filter.py
@brief Node that filters tracked agents from laser data for HATEB planner

@details This node subscribes to tracked agents and laser scan data, removes 
the agent detections from the scan, and publishes the filtered laser scan
used by the HATEB local planner.
@author Phani Teja Singamaneni
"""

import rospy
import sys
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan
from cohan_msgs.msg import TrackedAgents, TrackedSegmentType
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import math

## Some Global Variables
SCAN_TOPIC = "/base_scan"
MAP_FRAME = "map"
LASER_FRAME = "base_laser_link"
TRACKED_AGENTS_TOPIC = "/tracked_agents"

class AgentFilter(object):
    """
    @brief Filters tracked agents from laser scan data

    @details Subscribes to laser scan and tracked agents topics, removes agent 
    detections from the scan data, and publishes filtered scan for navigation
    """

    def __init__(self, ns):
        """
        @brief Initialize the AgentFilter node

        @param ns Namespace for the node (optional)
        """
        rospy.init_node('agent_filter')
        if ":=" in ns:
            self.ns_ = ""
        else:
            self.ns_ = ns
        self.rate = rospy.Rate(50.0)
        self.filtered_scan = LaserScan()
        self.segment_type = TrackedSegmentType.TORSO
        self.agents = []
        self.laser_transform = TransformStamped()
        self.got_scan = False
        self.laser_frame = LASER_FRAME

        ## Initialize tf2 transform listener
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)

        ## Adjusting the topics for handling namespaces
        laser_scan = SCAN_TOPIC
        tracked_agents = TRACKED_AGENTS_TOPIC
        if(self.ns_ != ""):
            laser_scan = "/"+self.ns_+SCAN_TOPIC
            tracked_agents = "/"+self.ns_+tracked_agents
            self.laser_frame = self.ns_ +"/" + LASER_FRAME


        rospy.Subscriber(laser_scan, LaserScan, self.laserCB)
        rospy.Subscriber(tracked_agents, TrackedAgents, self.agentsCB)
        self.laser_pub = rospy.Publisher("base_scan_filtered", LaserScan, queue_size=10)

        rospy.Timer(rospy.Duration(0.02), self.publishScan)

        # Keep the node alive
        rospy.spin()

    def laserCB(self, scan):
        """
        @brief Callback for laser scan messages

        @param scan LaserScan message containing raw scan data
        @details Processes incoming laser scan data to remove detections of tracked agents
        """
        filtered_scan = scan
        filtered_scan.ranges = list(scan.ranges)
        filtered_scan.header.stamp = rospy.Time.now()

        try:
            self.laser_transform = self.tf.lookup_transform(MAP_FRAME, self.laser_frame, rospy.Time(),rospy.Duration(5.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        if(self.laser_transform.header.frame_id != ''):
            laser_pose = self.laser_transform.transform.translation

            rot = self.laser_transform.transform.rotation
            r,p,y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            base_laser_dir = [np.cos(y), np.sin(y)]


            # Filtering agents from the scan
            for pose_type in self.agents:
                rh_vec = [pose_type[0].position.x - laser_pose.x, pose_type[0].position.y - laser_pose.y]
                sign = math.copysign(1, base_laser_dir[0]*-rh_vec[1] + base_laser_dir[1]*rh_vec[0])
                t_angle = scan.angle_max - scan.angle_min
                mid_angle = t_angle/2 - sign*np.arccos((base_laser_dir[0]*rh_vec[0]+base_laser_dir[1]*rh_vec[1])/(np.linalg.norm(rh_vec)))

                if math.isnan(mid_angle):
                    continue

                mid_idx = int((mid_angle)/scan.angle_increment)
                if(mid_idx>=len(scan.ranges)):
                    continue

                if pose_type[1] == 0:
                    r = 0.6
                else:
                    r = 0.4
                d = np.linalg.norm(rh_vec)
                mr = scan.ranges[mid_idx]

                if(mr<=(d-r)):
                    continue

                if(r<=d):
                    beta = np.arcsin(r/d)
                else:
                    beta = np.pi/2

                min_idx = int(np.floor((mid_angle-beta)/scan.angle_increment))
                max_idx = int(np.ceil((mid_angle+beta)/scan.angle_increment))

                for i in range(min_idx, max_idx):
                    if(i<len(scan.ranges)):
                        filtered_scan.ranges[i] = scan.range_max        

        #print (filtered_scan.ranges)
        self.filtered_scan = filtered_scan
        self.got_scan = True

    def agentsCB(self,msg):
        """
        @brief Callback for tracked agents messages

        @param msg TrackedAgents message containing detected agent information
        @details Updates the list of tracked agents with their poses and types
        """
        for agent in msg.agents:
            # if agent.type == 0:
            #     continue
            for segment in agent.segments:
                if(segment.type == self.segment_type):
                    if(len(self.agents)<agent.track_id):
                        self.agents.append([segment.pose.pose,agent.type])
                    else:
                        self.agents[agent.track_id-1] = [segment.pose.pose,agent.type]

    def publishScan(self, event):
        """
        @brief Timer callback to publish filtered laser scan

        @param event Timer event data (unused)
        @details Publishes the most recent filtered laser scan with updated timestamp
        """
        if(self.got_scan):
            self.filtered_scan.header.stamp = rospy.Time.now()
            self.laser_pub.publish(self.filtered_scan)


if __name__ == '__main__':
    if(len(sys.argv)<4):
        ns = ""
    else:
        ns = sys.argv[1]
    print(ns)
    hfilter = AgentFilter(ns = ns)
