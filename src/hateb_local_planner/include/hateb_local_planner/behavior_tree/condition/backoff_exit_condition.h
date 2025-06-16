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

// New
#include <agent_path_prediction/AgentsInfo.h>
#include <agent_path_prediction/agents_class.h>
#include <hateb_local_planner/backoff.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for managing backoff recovery behavior
 *
 * This class provides functionality to control when to exit the backoff recovery mode.
 * It monitors agent states and recovery progress to determine when to resume normal navigation.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class BackoffExitCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  BackoffExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  BackoffExitCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~BackoffExitCondition() override;

  /**
   * @brief Method called to evaluate the condition
   * @return SUCCESS if backoff recovery should be exited, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input and output ports for recovery condition
   * @return Ports list containing agents_info, backoff_ptr, nav_goal, agents_ptr as inputs,
   *         and recovery status as output
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<agent_path_prediction::AgentsInfo>("agents_info"), BT::InputPort<std::shared_ptr<Backoff>>("backoff_ptr"), BT::BidirectionalPort<geometry_msgs::PoseStamped>("nav_goal"),
            BT::InputPort<std::shared_ptr<agents::Agents>>("agents_ptr"), BT::OutputPort<bool>("recovery")};
  }

 private:
  /**
   * @brief Checks if the recovery behavior is complete
   * @return True if recovery is complete, false otherwise
   */
  bool isRecoveryComplete();

  // Blackboard entries
  agent_path_prediction::AgentsInfo agents_info_;         //!< Information about agents in the environment
  geometry_msgs::PoseStamped current_goal_;               //!< Current navigation goal
  std::shared_ptr<Backoff> backoff_ptr_ = nullptr;        //!< Pointer to backoff behavior handler
  std::shared_ptr<agents::Agents> agents_ptr_ = nullptr;  //!< Pointer to agents manager
  int stuck_agent_;                                       //!< ID of the agent that is stuck
  double dist_max_;                                       //!< Maximum distance threshold

  std::string name_;  //!< Name of the node

  bool started_;     //!< Flag indicating if recovery has started
  bool new_goal_;    //!< Flag indicating if a new goal was received
  bool backed_off_;  //!< Flag indicating if robot has backed off
};
};  // namespace hateb_local_planner