#!/usr/bin/env python3
#
# revision history: xzt
#  20210604 (TE): first version
#
# usage:
#
# This script is to convert the ground truth of pedestrian kinematics from Gazebo for Cohan.
# ------------------------------------------------------------------------------

import rospy
from std_msgs.msg import Header
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
from cohan_msgs.msg import TrackedAgents, TrackedSegment, TrackedAgent, TrackedSegmentType


class TrackAgent:
    # Constructor
    def __init__(self):
        # Initialize ROS objects
        self.agent_type = TrackedAgent.MOVING
        self.segment_type = TrackedSegmentType.TORSO

        self.ped_sub = rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.ped_callback)
        self.track_agent_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=10)
        rospy.loginfo_once("Converting Pedsim TrackedPersons to Cohan TrackedAgents")

    # Callback function for the path subscriber
    def ped_callback(self, peds_msg: TrackedPersons):
        header = Header()
        header.frame_id = "map"

        cohan_agents = TrackedAgents()
        cohan_agents.header = header

        # ped = TrackedPerson
        for ped in peds_msg.tracks:
            # for ped in peds_msg.tracks:
            agent = TrackedAgent()
            agent.track_id = ped.track_id
            agent.name = f"human{agent.track_id}"
            agent.type = self.agent_type
            # agent.radius = 0.1
            segment = TrackedSegment()
            segment.type = self.segment_type
            segment.pose = ped.pose
            segment.twist = ped.twist

            agent.segments.append(segment)

            cohan_agents.agents.append(agent)

        self.track_agent_pub.publish(cohan_agents)


if __name__ == "__main__":
    try:
        rospy.init_node("pedsim_to_cohan_agents")
        tp = TrackAgent()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
