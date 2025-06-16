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

#include <ros/ros.h>
#include <tf2/utils.h>

// Messages
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>

// New
#include <agent_path_prediction/AgentsInfo.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for checking if navigation goal is reached
 *
 * This class monitors the robot's position relative to the navigation goal and
 * provides functionality to determine when the goal has been successfully reached.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
// This node is not being used in BT tree. Not Tested very well. Use isGoalUpdated instead.
class IsGoalReached : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  IsGoalReached(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  IsGoalReached() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~IsGoalReached() override;

  /**
   * @brief Callback for receiving new navigation goals
   * @param goal_msg Message containing the new goal pose
   */
  void goalReceivedCB(const move_base_msgs::MoveBaseActionGoal& goal_msg);

  /**
   * @brief Callback for receiving goal status updates
   * @param result_msg Message containing the goal status result
   */
  void resultCB(const move_base_msgs::MoveBaseActionResult& result_msg);

  /**
   * @brief Method called to evaluate if goal is reached
   * @return SUCCESS if goal is reached or updated, FAILURE if still in progress
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input and output ports for condition evaluation
   * @return Ports list containing agents_info as input and nav_goal as output
   */
  static BT::PortsList providedPorts() {
    // This action has a single input port called "agents_info"
    return {BT::InputPort<agent_path_prediction::AgentsInfo>("agents_info"), BT::OutputPort<geometry_msgs::Pose>("nav_goal")};
  }

 private:
  /**
   * @brief Helper method to check if robot has reached goal position
   * @return True if robot is at goal position and orientation, false otherwise
   */
  bool goalReachedCheck();

  ros::Subscriber goal_sub_, status_sub_;  //!< Subscribers for goal and status updates
  bool goal_reached_;                      //!< Flag indicating if goal is reached
  bool updated_;                           //!< Flag indicating if goal was updated

  // Blackboard entries
  geometry_msgs::Pose goal_;                       //!< Current navigation goal pose
  agent_path_prediction::AgentsInfo agents_info_;  //!< Information about agents in the environment

  std::string name_;  //!< Name of the node
};
};  // namespace hateb_local_planner
