/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024-2025 LAAS-CNRS
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Author: Phani Teja Singamaneni
 *********************************************************************************/

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
