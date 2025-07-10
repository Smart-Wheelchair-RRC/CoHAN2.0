#!/usr/bin/env python3
"""
Software License Agreement (MIT License)

Copyright (c) 2024–2025 LAAS-CNRS

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


import rospy
import tf2_ros
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from cohan_msgs.msg import TrackedAgents, AgentPathArray, AgentPath, TrackedSegmentType
from cohan_msgs.srv import Optimize
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs

## These topics are for testing the robot's planner without any namespaces
NAME = "HATebStaticPlanVisualizer"
GET_PLAN_SRV = "/move_base/GlobalPlanner/make_plan"
OPTIMIZE_SRV = "/move_base/HATebLocalPlannerROS/optimize"
AGENTS_SUB = "/tracked_agents"
ROBOT_GOAL_SUB = "/clicked_point"
DEFAULT_AGENT_PART = TrackedSegmentType.TORSO

class StaticPlanVisualization:
    def __init__(self):
        self.initialized = False
        self.predict_behind_robot = True
        self.got_robot_plan = False
        self.got_agent_plan = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize empty variables
        self.agents_start_poses = []
        self.agents_goals = []
        self.tracked_agents = None
        self.robot_start_pose = PoseStamped()
        self.robot_goal = PoseStamped()
        self.robot_plan = Path()
        self.agents_plans = AgentPathArray()
        self.robot_to_map_tf = None
        self.ns = ""

    def initialize(self):
        if not self.initialized:
            # Get namespace parameter
            self.ns = rospy.get_param('~ns', '')

            # Setup service names with namespace
            get_plan_srv_name = GET_PLAN_SRV
            optimize_srv_name = OPTIMIZE_SRV
            if self.ns:
                get_plan_srv_name = f"/{self.ns}{get_plan_srv_name}"
                optimize_srv_name = f"/{self.ns}{optimize_srv_name}"

            # Setup service clients
            self.get_plan_client = rospy.ServiceProxy(get_plan_srv_name, GetPlan, persistent=True)
            self.optimize_client = rospy.ServiceProxy(optimize_srv_name, Optimize, persistent=True)

            # Setup subscribers
            rospy.Subscriber(AGENTS_SUB, TrackedAgents, self.update_start_poses)
            rospy.Subscriber(ROBOT_GOAL_SUB, PointStamped, self.update_goals_and_optimize)

            # Setup service
            rospy.Service('optimize_srv', SetBool, self.optimize_srv)
            
            self.initialized = True

    def update_start_poses(self, tracked_agents):
        self.agents_start_poses = []
        self.tracked_agents = tracked_agents

        # Process agent poses
        for agent in tracked_agents.agents:
            for segment in agent.segments:
                if segment.type == DEFAULT_AGENT_PART:
                    hum_pose = PoseStamped()
                    hum_pose.pose = segment.pose.pose
                    hum_pose.header.frame_id = "map"
                    hum_pose.header.stamp = rospy.Time.now()
                    self.agents_start_poses.append(hum_pose)

        try:
            base = "base_footprint"
            if self.ns:
                base = f"{self.ns}/{base}"
            self.robot_to_map_tf = self.tf_buffer.lookup_transform("map", base, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            rospy.sleep(1.0)
            return

        # Update robot start pose from tf
        self.robot_start_pose.header = self.robot_to_map_tf.header
        self.robot_start_pose.pose = Pose()
        self.robot_start_pose.pose.position.x = self.robot_to_map_tf.transform.translation.x
        self.robot_start_pose.pose.position.y = self.robot_to_map_tf.transform.translation.y
        self.robot_start_pose.pose.position.z = self.robot_to_map_tf.transform.translation.z
        self.robot_start_pose.pose.orientation = self.robot_to_map_tf.transform.rotation

    def update_goals_and_optimize(self, robot_goal_point):
        self.robot_goal.pose.position = robot_goal_point.point
        self.robot_goal.header = robot_goal_point.header
        self.robot_goal.pose.orientation = self.robot_start_pose.pose.orientation
        now = rospy.Time.now()

        # Get global robot plan
        try:
            robot_plan_resp = self.get_plan_client(
                start=self.robot_start_pose,
                goal=self.robot_goal,
                tolerance=0.1  # Add a reasonable tolerance value
            )
            self.got_robot_plan = len(robot_plan_resp.plan.poses) > 0
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return

        # Process agent plans
        hum_path_arr = AgentPathArray()
        hum_path_arr.header.frame_id = "map"
        hum_path_arr.header.stamp = now
        
        self.agents_goals = []
        
        for idx, agent in enumerate(self.tracked_agents.agents):
            if agent.track_id == 1 and self.predict_behind_robot:
                # Calculate position behind robot
                behind_pose = Pose()
                behind_pose.position.x = -0.5  # 0.5m behind
                behind_pose_stamped = PoseStamped()
                behind_pose_stamped.pose = behind_pose
                behind_pose_transformed = tf2_geometry_msgs.do_transform_pose(
                    behind_pose_stamped, self.robot_to_map_tf)

                agent_goal = PoseStamped()
                agent_goal.header.frame_id = "map"
                agent_goal.header.stamp = now
                agent_goal.pose = behind_pose_transformed.pose
                self.agents_goals.append(agent_goal)

                # Get plan for agent
                try:
                    agent_plan_resp = self.get_plan_client(
                        start=self.agents_start_poses[idx],
                        goal=agent_goal,
                        tolerance=0.1  # Add a reasonable tolerance value
                    )
                    self.got_agent_plan = len(agent_plan_resp.plan.poses) > 0
                    
                    if self.got_agent_plan:
                        temp = AgentPath()
                        temp.id = agent.track_id
                        temp.path = agent_plan_resp.plan
                        hum_path_arr.paths.append(temp)
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                    continue
            else:
                self.agents_goals.append(self.agents_start_poses[idx])

        self.agents_plans = hum_path_arr
        self.robot_plan = robot_plan_resp.plan

        if self.got_agent_plan and self.got_robot_plan:
            self.optimize_plans(True)

    def optimize_plans(self, include_agent_plans):
        # Prepare agent plan array
        if not include_agent_plans:
            # Create minimal agent plan array with just start poses
            hum_path_arr = AgentPathArray()
            hum_path_arr.header.frame_id = "map"
            hum_path_arr.header.stamp = rospy.Time.now()
            
            temp = AgentPath()
            temp.id = 1
            temp.path.poses.append(self.agents_start_poses[0])
            hum_path_arr.paths.append(temp)
        else:
            hum_path_arr = self.agents_plans

        try:
            # Call optimize service with named parameters
            optim_resp = self.optimize_client(
                robot_plan=self.robot_plan,
                agent_plan_array=hum_path_arr
            )
            if optim_resp.success:
                rospy.loginfo(optim_resp.message)
                return optim_resp
            else:
                rospy.loginfo("Optimization failed!")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def optimize_srv(self, req):
        response = SetBoolResponse()
        
        if self.got_agent_plan and self.got_robot_plan:
            result = self.optimize_plans(req.data)
            if result:
                response.success = True
                response.message = result.message
            else:
                response.success = False
                response.message = "Optimization failed..!!"
        else:
            response.success = False
            response.message = "No valid plans available"
            
        return response

def main():
    rospy.init_node(NAME)
    
    static_viz = StaticPlanVisualization()
    static_viz.initialize()
    
    rospy.spin()

if __name__ == '__main__':
    main()
