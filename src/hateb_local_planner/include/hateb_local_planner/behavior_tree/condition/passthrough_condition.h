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
#include <cohan_msgs/PassageType.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for checking passthrough scenarios
 *
 * This class evaluates whether the robot is at a passage situation that requires special handling.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class PassThroughCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  PassThroughCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  PassThroughCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~PassThroughCondition() override;

  /**
   * @brief Method called to evaluate passage conditions
   * @return SUCCESS if in door/pillar passage, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input port for passage type
   * @return Ports list containing passage type as input
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<int8_t>("passage_type")};  //!< Type of passage (door, pillar, open, wall)
  }

 private:
  int8_t psg_type_;  //!< Current passage type being evaluated

  std::string name_;  //!< Name of the node
};
};  // namespace hateb_local_planner
