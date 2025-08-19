#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class HumanController:
    def __init__(self, x=None, y=None):
        self.ps = rospy.Subscriber('/initial_human_pose', PoseStamped, self.pose_callback)
        self.ts = rospy.Subscriber('/human_cmd_vel', Twist, self.velocity_callback)
        self.pose_pub = rospy.Publisher('/cylinder_with_arrow/pose_stamped', PoseStamped, queue_size=10)
        self.base_pose_pub = rospy.Publisher('/human_pose', Odometry, queue_size=10)
        _ = rospy.Timer(rospy.Duration(0.1), self.pose_publisher)

        ## Set the initial pose of the agent (can be done manually or through the topic)
        self.human_pose = PoseStamped()
        if x is None or y is None:
            self.human_pose.header.frame_id = "map"
            self.human_pose.pose.position.x = 0
            self.human_pose.pose.position.y = 0
            self.human_pose.pose.orientation.w = 1.0
        else:
            self.human_pose.header.frame_id = "map"
            self.human_pose.pose.position.x = x
            self.human_pose.pose.position.y = y
            self.human_pose.pose.orientation.w = 1.0

        ## Initial Velocity
        self.human_velocity = Twist()

        rospy.sleep(0.5)
        self.pose_pub.publish(self.human_pose)

    def pose_callback(self, msg):
        rospy.loginfo("Received human pose: %s", msg)
        self.human_pose = msg
        self.pose_pub.publish(self.human_pose)

    def velocity_callback(self, msg):
        rospy.loginfo("Received human velocity: %s", msg)
        self.human_velocity = msg
        ## Update the human pose based on the velocity
        self.move_human()
        ## Publish the updated human pose
        self.pose_pub.publish(self.human_pose)

    def move_human(self):
        """
        Update the human pose based on the current velocity.
        """
        dt = 0.1  # Time step
        self.human_pose.pose.position.x += self.human_velocity.linear.x * dt
        self.human_pose.pose.position.y += self.human_velocity.linear.y * dt
        _, _, yaw = euler_from_quaternion([self.human_pose.pose.orientation.x,
                                            self.human_pose.pose.orientation.y,
                                            self.human_pose.pose.orientation.z,
                                            self.human_pose.pose.orientation.w])
        yaw += self.human_velocity.angular.z * dt
        quat = quaternion_from_euler(0, 0, yaw)
        self.human_pose.pose.orientation.x = quat[0]
        self.human_pose.pose.orientation.y = quat[1]
        self.human_pose.pose.orientation.z = quat[2]
        self.human_pose.pose.orientation.w = quat[3]

    def pose_publisher(self, event):
        """
        Publish the current human pose + velocity at a regular interval.
        """
        # Publish the base pose + velocity as Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose = self.human_pose.pose
        odom_msg.twist.twist = self.human_velocity
        self.base_pose_pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('human_controller')
    if len(sys.argv) > 2:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        controller = HumanController(x, y)
    else:
        controller = HumanController(5, 2)
    rospy.spin()
