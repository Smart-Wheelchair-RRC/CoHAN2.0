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

#include <hateb_local_planner/behavior_tree/bt_core.h>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for checking if navigation goal has been updated
 *
 * This class monitors updates to the robot's navigation goal and provides functionality
 * to detect when a new goal has been set. This is important for handling goal changes
 * during navigation and recovery behaviors.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class IsGoalUpdated : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  IsGoalUpdated(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  IsGoalUpdated() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~IsGoalUpdated() override;

  /**
   * @brief Method called to evaluate if goal has been updated
   * @return SUCCESS if goal has been updated and not in recovery, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input ports for condition evaluation
   * @return Ports list containing goal_update and recovery status as inputs
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("goal_update"),  //!< Flag indicating if goal was updated
            BT::InputPort<bool>("recovery")};    //!< Flag indicating if in recovery mode
  }

 private:
  std::string name_;  //!< Name of the node
};
};  // namespace hateb_local_planner
