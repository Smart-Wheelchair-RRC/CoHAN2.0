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
#include <hateb_local_planner/behavior_tree/bt_core.h>

#include <mutex>

namespace hateb_local_planner {

/**
 * @brief Condition node that checks if an obstacle (human) has stopped in the environment
 *
 * This class implements a behavior tree condition that monitors the velocity state of
 * nearby human agents and determines if any have come to a stop. It is used to detect
 * situations where the robot's path may be blocked by a stationary human obstacle.
 */
class VelObsExitCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for the VelObsExitCondition node
   * @param condition_name Name of the condition node
   * @param conf Node configuration containing the input/output ports
   */
  VelObsExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  VelObsExitCondition() = delete;

  /**
   * @brief Destructor
   */
  ~VelObsExitCondition() override;

  /**
   * @brief Main execution tick of the condition node
   * @return NodeStatus SUCCESS if a human has stopped, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines the input and output ports for the condition node
   * @return PortsList containing the node's ports configuration
   */
  static BT::PortsList providedPorts() {
    // This action has a single input port called "agents_info"
    return {BT::InputPort<agent_path_prediction::AgentsInfo>("agents_info"), BT::InputPort<std::shared_ptr<agents::Agents>>("agents_ptr"), BT::OutputPort<int>("stuck_agent")};
  }

 private:
  /**
   * @brief Checks if any human agent in the environment has stopped moving
   * @return true if a human has stopped, false otherwise
   */
  bool hasHumanStopped();

  // bool isHumanPlaying(); // Add this later

  // Blackboard entries
  agent_path_prediction::AgentsInfo agents_info_;  //!< Current information about all agents in the environment
  std::shared_ptr<agents::Agents> agents_ptr_;     //!< Pointer to the agents management class

  std::string name_;         //!< Name of this behavior tree node
  int nearest_human_id_;     //!< ID of the nearest human agent
  int t_stuck_;              //!< Time counter for how long an agent has been stuck
  std::mutex agents_mutex_;  //!< Mutex to protect concurrent access to agent data
};
};  // namespace hateb_local_planner