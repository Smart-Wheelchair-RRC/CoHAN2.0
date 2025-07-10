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
