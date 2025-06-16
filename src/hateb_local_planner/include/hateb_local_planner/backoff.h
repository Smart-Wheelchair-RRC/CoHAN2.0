/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025 LAAS/CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/
#ifndef BACKOFF_H_
#define BACKOFF_H_

#include <actionlib/client/simple_action_client.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>

namespace hateb_local_planner {

/**
 * @brief Class implementing backoff behavior for robot navigation
 *
 * The Backoff class provides recovery behavior when the robot encounters
 * a human in a place where both agents (human and robot) get stuck without
 * any progress towards to goal. It implements methods to calculate and
 * execute backoff maneuvers to help the robot recover from such situations.
 */
class Backoff {
 public:
  /**
   * @brief Constructor
   */
  Backoff(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destructor
   */
  ~Backoff();

  /**
   * @brief Initializes the backoff behavior with a costmap
   * @param costmap_ros Pointer to the costmap ROS wrapper
   */
  void initialize(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Sets a new backoff goal position
   * @param goal The goal pose to back off to
   * @return True if goal was successfully set
   */
  bool setbackGoal(geometry_msgs::PoseStamped goal);

  /**
   * @brief Checks if backoff behavior has timed out
   * @return True if timeout occurred
   */
  bool timeOut();

  /**
   * @brief Initiates the recovery behavior
   * @return True if recovery was successfully started
   */
  bool startRecovery();

  /**
   * @brief Checks if a new goal has been received
   * @return True if there is a new goal
   */
  bool checkNewGoal() const { return new_goal_; }

  /**
   * @brief Checks if the backoff goal position has been reached
   * @param robot_pose Current pose of the robot
   * @return True if goal has been reached
   */
  bool isBackoffGoalReached(geometry_msgs::Pose2D& robot_pose) const;

  /**
   * @brief Initializes the grid offsets for obstacle checking
   *
   * Sets up two grids of points (left and right) around the robot's circumference
   * used for checking nearby obstacles when planning backoff maneuvers.
   *
   * @param r Robot's circumscribed radius
   */
  void initializeOffsets(double r) {
    robot_circumscribed_radius_ = r;
    const double right_grid_offsets[4][2] = {{r, r}, {r, -r}, {-r, -r}, {-r, r}};  //{{r, -r}, {r, -3 * r}, {-r, -3 * r}, {-r, -r}};
    const double left_grid_offsets[4][2] = {{r, r}, {r, -r}, {-r, -r}, {-r, r}};   //{{r, 3 * r}, {r, r}, {-r, r}, {-r, 3 * r}};

    for (const auto* offset : right_grid_offsets) {
      geometry_msgs::Point p;
      p.x = offset[0];
      p.y = offset[1];
      p.z = 0;
      right_grid_.push_back(p);
    }

    for (const auto* offset : left_grid_offsets) {
      geometry_msgs::Point p;
      p.x = offset[0];
      p.y = offset[1];
      p.z = 0;
      left_grid_.push_back(p);
    }
  }

 private:
  /**
   * @brief Callback for processing new goal messages
   * @param goal Pointer to the received goal message
   */
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

  /**
   * @brief Normalizes an angle to the range [-π, π]
   * @param angle_radians Angle to normalize (in radians)
   * @return Normalized angle in radians
   */
  static double normalize_angle(double angle_radians) {
    // Use ceres::floor because it is specialized for double and Jet types.
    double two_pi = 2.0 * M_PI;
    return angle_radians - (two_pi * std::floor((angle_radians + (M_PI)) / two_pi));
  }

  /**
   * @brief Loads ROS parameters from the node handle
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);

  std::string map_frame_;           //!< Name of the map frame
  std::string footprint_frame_;     //!< Name of the robot's footprint frame
  std::string ns_;                  //!< Namespace for the node
  std::string publish_goal_topic_;  //!< Topic name for publishing backoff goals
  std::string current_goal_topic_;  //!< Topic name for current goal
  std::string get_plan_srv_name_;   //!< Service name for path planning
  double backoff_timeout_;          //!< Maximum allowed time for backoff maneuver

  geometry_msgs::PoseStamped goal_;      //!< Current navigation goal
  geometry_msgs::PoseStamped old_goal_;  //!< Previous navigation goal

  tf::TransformListener tf_;  //!< Transform listener for coordinate transformations

  costmap_2d::Costmap2DROS* costmap_ros_;  //!< Pointer to the costmap ROS wrapper
  costmap_2d::Costmap2D* costmap_;         //!< Pointer to the 2D costmap
  double robot_circumscribed_radius_;      //!< Radius of circle encompassing the robot

  // ROS communication members
  ros::Publisher goal_pub_;             //!< Publisher for sending backoff goal poses
  ros::Publisher poly_pub_l_;           //!< Publisher for visualizing left grid points
  ros::Publisher poly_pub_r_;           //!< Publisher for visualizing right grid points
  ros::Subscriber goal_sub_;            //!< Subscriber for receiving navigation goals
  ros::ServiceClient get_plan_client_;  //!< Service client for requesting path plans

  geometry_msgs::Transform start_pose_;  //!< Initial robot pose when backoff maneuver starts
  bool visualize_backoff_;               //!< Enable visualization of backoff grids
  bool self_published_;                  //!< Whether the goal was published by this node
  bool new_goal_;                        //!< Whether a new navigation goal was received

  ros::Time last_time_;  //!< Time of the last update
  // ros::Time last_rot_time_;   //!< Time of the last rotation movement
  // ros::Time last_goal_time_;  //!< Time when the last goal was received

  tf::Transform start_pose_tr_;           //!< Initial pose stored as transform
  tf::StampedTransform robot_to_map_tf_;  //!< Transform from robot base to map frame

  std::shared_ptr<base_local_planner::CostmapModel> costmap_model_;  //!< Model for collision checking with costmap
  geometry_msgs::PoseStamped backoff_goal_;                          //!< Goal pose for backoff maneuver

  std::vector<geometry_msgs::Point> left_grid_;   //!< Grid points for left obstacle checks
  std::vector<geometry_msgs::Point> right_grid_;  //!< Grid points for right obstacle checks
};

}  // namespace hateb_local_planner
#endif  // BACKOFF_H_
