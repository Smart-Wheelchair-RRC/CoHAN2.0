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

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from nav_msgs.msg import Odometry


class MocapAgents(object):
    """
    Bridge between CoHAN Sim agents and CoHAN tracked_agents message.

    Subscribes to simulated human and robot odometry topics, converts them to TrackedAgents messages,
    and publishes them for use by CoHAN Navigation. Handles both human and robot agents, synchronizes their states,
    and assigns appropriate segment and agent types.
    """

    def __init__(self, num_hum, ns_):
        """
        Initialize the MocapAgents bridge.

        Args:
            num_hum (int): Number of human agents in the simulation.
            ns_ (str): Namespace for the current agent (empty string for robot).
        """
        self.num_hum = num_hum
        self.ns = ns_
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()

    def AgentsPub(self):
        """
        Set up ROS node, subscribers, and publisher for tracked agents.
        Synchronizes human agent odometry and starts publishing tracked_agents messages.
        """
        rospy.init_node('mujoco_mocap_agent', anonymous=True)
        agent_sub = rospy.Subscriber("/human_pose", Odometry, self.AgentCB)
        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentCB(self, msg):
        """
        Callback for synchronized human agent odometry message.
        Converts odometry to TrackedAgent message and updates the tracked_agents list.

        Args:
            *msg: Odometry messages for human agent.
        """
        if len(msg) != self.num_hum:
            return
    
        tracked_agents = TrackedAgents()
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.pose.pose
        agent_segment.twist.twist = msg.twist.twist
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.HUMAN
        tracked_agent.name = "human1_mocap"
        tracked_agent.segments.append(agent_segment)
        tracked_agents.agents.append(tracked_agent)
        if(tracked_agents.agents):
            self.agents = tracked_agents

    def publishAgents(self, event):
        """
        Publishes the current tracked_agents message. Ensure to publish agent data continuously.
        Assigns track IDs and sets the header fields.

        Args:
            event (rospy.TimerEvent): The timer event.
        """
        self.agents.header.stamp = rospy.Time.now()
        self.agents.header.frame_id = "map"
        self.agents.agents[0].track_id = 1
        self.tracked_agents_pub.publish(self.agents)
 


if __name__ == '__main__':
    nh = sys.argv[1]
    if(len(sys.argv)<5):
        ns=""
    else:
        ns = sys.argv[2]
    agents = MocapAgents(num_hum=int(nh), ns_= ns)
    agents.AgentsPub()
