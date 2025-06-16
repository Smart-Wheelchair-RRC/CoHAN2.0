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

#include <hateb_local_planner/PlanningMode.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace hateb_local_planner {

/**
 * @brief Class implementing a behavior tree action node to set planning mode
 *
 * This class provides functionality to set planning modes in CoHAN.
 * It inherits from StatefulActionNodeROS to integrate with ROS and behavior tree framework.
 */
class SetMode : public StatefulActionNodeROS {
 public:
  /**
   * @brief Constructor with ROS node handle and behavior tree configuration
   * @param nh The ROS node handle for communication
   * @param name Name of the behavior tree node
   * @param config Configuration for the behavior tree node
   */
  SetMode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  SetMode() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~SetMode() override;

  /**
   * @brief Defines input and output ports for the behavior tree node
   * @return Ports list containing plan_type and predict_type as inputs and mode as output
   */
  static BT::PortsList providedPorts() {
    // define the input and output ports
    return {BT::InputPort<std::string>("plan_type"), BT::InputPort<std::string>("predict_type"), BT::OutputPort<ModeInfo>("mode")};
  }

  /**
   * @brief Method called when the node is first ticked
   * @return Status of the node after starting
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Method called while the node is running
   * @return Current status of the node
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Method called when the node is halted
   */
  void onHalted() override;

 private:
  std::string name_;                  //!< Name of the behavior tree node
  ros::Publisher planning_mode_pub_;  //!< Publisher for planning mode messages

  // BT Tree main
  std::string plan_type_;     //!< Type of planning mode to use
  std::string predict_type_;  //!< Type of prediction to use
  ModeInfo p_msg_;            //!< Message containing mode information
};
};  // namespace hateb_local_planner