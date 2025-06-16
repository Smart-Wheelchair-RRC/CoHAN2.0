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

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for checking single band exit criteria
 *
 * This class provides functionality to determine if a single band planning mode
 * should be exited based on agent information and distance thresholds.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class SingleBandExitCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  SingleBandExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  SingleBandExitCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~SingleBandExitCondition() override;

  /**
   * @brief Method called to evaluate the condition
   * @return Status indicating whether condition is met
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input ports for condition evaluation
   * @return Ports list containing agents_info and dist_max as inputs
   */
  static BT::PortsList providedPorts() {
    // This action has a single input port called "agents_info"
    return {BT::InputPort<agent_path_prediction::AgentsInfo>("agents_info"), BT::InputPort<double>("dist_max")};
  }

 private:
  // Blackboard entries
  agent_path_prediction::AgentsInfo agents_info_;  //!< Information about agents in the environment
  double dist_max_;                                //!< Maximum distance threshold for band exit condition

  // name of the node
  std::string name_;  //!< Name of the condition node
};
};  // namespace hateb_local_planner
