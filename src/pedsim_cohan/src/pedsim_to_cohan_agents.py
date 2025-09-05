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
import tf
import numpy as np
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
from cohan_msgs.msg import TrackedAgents, TrackedSegment, TrackedAgent, TrackedSegmentType
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped


class TrackAgent:
    # Constructor
    def __init__(self):
        # Initialize ROS objects
        self.agent_type = TrackedAgent.MOVING
        self.segment_type = TrackedSegmentType.TORSO
        self.num_closest_agents = rospy.get_param("num_closest_agents", default=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ped_sub = rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.ped_callback)
        self.get_state_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.track_agent_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=10)
        rospy.loginfo_once("Converting Pedsim TrackedPersons to Cohan TrackedAgents")

    def get_robot_states_gazebo(self):
        """
        Obtaining robot current position (x, y, theta)
        :param x x-position of the robot
        :param y y-position of the robot
        :param theta theta-position of the robot
        """
        robot = None
        # get robot and pedestrian states:
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            # get robot state:
            model_name = "mobile_base"
            relative_entity_name = "world"
            robot = self.get_state_service(model_name, relative_entity_name)
            t = np.array([robot.pose.position.x, robot.pose.position.y])

            q = (
                robot.pose.orientation.x,
                robot.pose.orientation.y,
                robot.pose.orientation.z,
                robot.pose.orientation.w,
            )
            return self.get_tf_matrix(t, q)
        except rospy.ServiceException:
            rospy.logwarn("/gazebo/get_model_state service call failed")

    def get_base_link_to_map_tf(self):
        try:
            # lookup transform from map → base_link
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                "map",  # source frame
                rospy.Time(0),  # get the latest available transform
                rospy.Duration(1.0),  # timeout
            )

            # Extract translation
            t = np.array([trans.transform.translation.x, trans.transform.translation.y])

            # Extract quaternion
            q = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            )

            return self.get_tf_matrix(t, q)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF error: {e}")

    def get_tf_matrix(self, t, q):
        pos = np.zeros(3)
        pos[:2] = t

        (_, _, pos[2]) = tf.transformations.euler_from_quaternion(q)

        T = np.array(
            [
                [np.cos(pos[2]), -np.sin(pos[2]), pos[0]],
                [np.sin(pos[2]), np.cos(pos[2]), pos[1]],
                [0, 0, 1],
            ]
        )

        T_inv = np.linalg.inv(T)

        return T_inv

    def ped_callback(self, peds_msg: TrackedPersons):
        if self.num_closest_agents > -1 or self.num_closest_agents > len(peds_msg.tracks):
            self.closest_ped_callback(peds_msg)
        else:
            self.all_ped_callback(peds_msg)

    # Callback function for the path subscriber
    def all_ped_callback(self, peds_msg: TrackedPersons):
        gazebo_T_base = self.get_robot_states_gazebo()
        base_T_map = self.get_base_link_to_map_tf()

        if np.any(gazebo_T_base) and np.any(base_T_map):
            # get pedestrian poses and velocities:
            gazebo_T_map = gazebo_T_base @ base_T_map

            cohan_agents = TrackedAgents()
            cohan_agents.header.frame_id = "map"
            cohan_agents.header.stamp = rospy.Time.now()

            for ped in peds_msg.tracks:
                agent = TrackedAgent()
                agent.track_id = ped.track_id
                agent.name = f"human{agent.track_id}"
                agent.type = self.agent_type
                #
                # relative positions and velocities:
                ped_pos = np.array([ped.pose.pose.position.x, ped.pose.pose.position.y, 1])
                ped_vel = np.array([ped.twist.twist.linear.x, ped.twist.twist.linear.y])
                ped_pos_in_robot = np.matmul(gazebo_T_map, ped_pos.T)
                ped_vel_in_robot = np.matmul(gazebo_T_map[:2, :2], ped_vel.T)

                segment = TrackedSegment()
                segment.type = self.segment_type
                segment.pose = ped.pose
                segment.twist = ped.twist
                segment.pose.pose.position.x = ped_pos_in_robot[0]
                segment.pose.pose.position.y = ped_pos_in_robot[1]
                segment.twist.twist.linear.x = ped_vel_in_robot[0]
                segment.twist.twist.linear.y = ped_vel_in_robot[1]

                agent.segments.append(segment)

                cohan_agents.agents.append(agent)

            self.track_agent_pub.publish(cohan_agents)

    def closest_ped_callback(self, peds_msg: TrackedPersons):
        # gazebo_T_map = self.get_gazebo_to_map_tf()
        gazebo_T_base = self.get_robot_states_gazebo()
        base_T_map = self.get_base_link_to_map_tf()

        if np.any(gazebo_T_base) and np.any(base_T_map):
            # get pedestrian poses and velocities:
            pose_array = []
            vel_array = []
            track_ids = []

            for ped in peds_msg.tracks:
                #
                # relative positions and velocities:
                ped_pos = np.array([ped.pose.pose.position.x, ped.pose.pose.position.y, 1])
                ped_vel = np.array([ped.twist.twist.linear.x, ped.twist.twist.linear.y])
                ped_pos_in_robot = np.matmul(gazebo_T_base, ped_pos.T)
                ped_vel_in_robot = np.matmul(gazebo_T_base[:2, :2], ped_vel.T)

                pose_array.append(ped_pos_in_robot)
                vel_array.append(ped_vel_in_robot)
                track_ids.append(ped.track_id)

            pose_array = np.array(pose_array)
            vel_array = np.array(vel_array)
            track_ids = np.array(track_ids)
            closest_indices = self.get_closest_pose_indices(pose_array)

            close_poses = pose_array[closest_indices]

            close_vels = vel_array[closest_indices]
            tracks = track_ids[closest_indices]

            cohan_agents = TrackedAgents()
            cohan_agents.header.frame_id = "map"
            cohan_agents.header.stamp = rospy.Time.now()

            for i in range(self.num_closest_agents):
                agent = TrackedAgent()
                agent.track_id = tracks[i]
                agent.name = f"human{agent.track_id}"
                agent.type = self.agent_type

                # pos = np.array([close_poses[i, 0], close_poses[i, 1], 1])
                pos = np.matmul(base_T_map, close_poses[i].T)

                vel = np.matmul(base_T_map[:2, :2], close_vels[i].T)

                segment = TrackedSegment()
                segment.type = self.segment_type
                segment.pose.pose.position.x = pos[0]
                segment.pose.pose.position.y = pos[1]
                segment.twist.twist.linear.x = vel[0]
                segment.twist.twist.linear.y = vel[1]

                agent.segments.append(segment)

                cohan_agents.agents.append(agent)

            self.track_agent_pub.publish(cohan_agents)

    def get_closest_pose_indices(self, poses):
        dist = np.hypot(poses[:, 0], poses[:, 1])
        # print(dist.shape)
        indices = np.argsort(dist)
        return indices[0 : self.num_closest_agents]


if __name__ == "__main__":
    try:
        rospy.init_node("pedsim_to_cohan_agents")
        tp = TrackAgent()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
