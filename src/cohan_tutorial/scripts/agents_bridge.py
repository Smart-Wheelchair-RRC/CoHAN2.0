#!/usr/bin/env python3

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from nav_msgs.msg import Odometry
import message_filters


class SimAgents(object):
    """
    Bridge between CoHAN Sim agents and CoHAN tracked_agents message.

    Subscribes to simulated human base_pose_groundtruth topics, converts them to TrackedAgents messages,
    and publishes them for use by CoHAN Navigation.
    """

    def __init__(self, num_hum):
        """
        Initialize the SimAgents bridge.

        Args:
            num_hum (int): Number of human agents in the simulation.
        """
        self.num_hum = num_hum
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()

    def AgentsPub(self):
        """
        Set up ROS node, subscribers, and publisher for tracked agents.
        Synchronizes human agent base_pose_groundtruth messages, and starts publishing tracked_agents messages.
        """
        rospy.init_node('Sim_Agents', anonymous=True)
        agent_sub = []

        for agent_id in range(1,self.num_hum+1):
            name = 'human'+str(agent_id)
            agent_sub.append(message_filters.Subscriber("/" + name + "/base_pose_ground_truth", Odometry))

        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        pose_msg = message_filters.TimeSynchronizer(agent_sub, 10)
        pose_msg.registerCallback(self.AgentsCB)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,*msg):
        """
        Callback for synchronized human agent odometry messages.
        Converts odometry to TrackedAgent messages and updates the tracked_agents list.

        Args:
            *msg: Synchronized odometry messages for human agents.
        """
        if len(msg) != self.num_hum:
            return
    
        tracked_agents = TrackedAgents()
        for agent_id in range(1,self.num_hum+1):
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose = msg[agent_id-1].pose.pose
            agent_segment.twist.twist = msg[agent_id-1].twist.twist
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human"+str(agent_id)
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
        if(tracked_agents.agents):
            self.agents = tracked_agents
            self.sig_1 = True

    def publishAgents(self, event):
        """
        Publishes the current tracked_agents message if both human and robot data are available.
        Assigns track IDs and sets the header fields.

        Args:
            event (rospy.TimerEvent): The timer event.
        """
        self.agents.header.stamp = rospy.Time.now()
        self.agents.header.frame_id = "map"
        for agent_id in range(0, len(self.agents.agents)):
            self.agents.agents[agent_id].track_id = agent_id+1
        self.tracked_agents_pub.publish(self.agents)


if __name__ == '__main__':
    num_hum = sys.argv[1]
    agents = SimAgents(num_hum=int(num_hum))
    agents.AgentsPub()
