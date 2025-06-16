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

#ifndef BT_CORE_HH_
#define BT_CORE_HH_

/**
 * @brief Core header file for behavior tree implementation in HATEB local planner
 *
 * This file contains the core definitions and components for implementing behavior trees
 * in the HATEB (Human-Aware Timed Elastic Band) local planner. It provides classes and
 * utilities for creating and managing ROS-integrated behavior tree nodes.
 */

#include <ros/ros.h>

#include "behaviortree_cpp/bt_factory.h"

/**
 * @brief Constants for numerical precision and distance thresholds
 */
#define EPS 0.01       // Small epsilon value for floating-point comparisons
#define DIST_EPS 0.06  // Distance threshold for proximity checks

/**
 * @brief Debug printing configuration and macros
 * When PRINT is enabled, provides colored console output for different message types:
 * - BT_INFO: Regular informational messages
 * - BT_WARN: Warning messages (yellow)
 * - BT_ERROR: Error messages (red)
 */
#define BTPRINT 0

#if BTPRINT
#define BT_INFO(x, y) std::cout << "BT_INFO: " << x << " -> " << y << std::endl;

#define BT_WARN(x, y)                                           \
  std::cout << "\033[33m";                                      \
  std::cout << "BT_WARNING: " << x << " -> " << y << std::endl; \
  std::cout << "\033[0m";

#define BT_ERROR(x, y)                                        \
  std::cout << "\033[31m";                                    \
  std::cout << "BT_ERROR: " << x << " -> " << y << std::endl; \
  std::cout << "\033[0m";

#else
#define BT_INFO(x, y)
#define BT_WARN(x, y)
#define BT_ERROR(x, y)
#endif

/**
 * @brief Normalizes an angle to the range [-π, π]
 * @param angle_radians The input angle to normalize
 * @return The normalized angle in radians
 */
inline double normalize_angle(double angle_radians) { return angle_radians - (2.0 * M_PI * std::floor((angle_radians + (M_PI)) / (2.0 * M_PI))); }

/*This part of the code is inspired from here: https://github.com/BehaviorTree/BehaviorTree.ROS (bt_action_node.hh)*/
namespace hateb_local_planner {

/**
 * @brief Base class for stateful action nodes in the behavior tree
 *
 * This class provides the foundation for creating ROS-integrated behavior tree nodes
 * that maintain state between executions. It inherits from BT::StatefulActionNode
 * and provides integration with ROS node handles.
 */
class StatefulActionNodeROS : public BT::StatefulActionNode {
 protected:
  /**
   * @brief Constructor initializes the node with ROS and BT configurations
   * @param nh ROS node handle for communication
   * @param name Name of the behavior tree node
   * @param conf Configuration for the behavior tree node
   */
  StatefulActionNodeROS(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf), node_(nh) {}

 public:
  // using BaseClass = StatefulActionNodeROS<ActionT>;
  // using ActionType = ActionT;

  StatefulActionNodeROS() = delete;

  ~StatefulActionNodeROS() override = default;

  /**
   * @brief Defines the input/output ports for the node
   * @return List of ports including the action_name input port
   */
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action_name")}; }

  /**
   * @brief Called when the node starts execution
   * @return Status of the node after starting
   */
  BT::NodeStatus onStart() override = 0;

  /**
   * @brief Called during node execution
   * @return Current status of the node
   */
  BT::NodeStatus onRunning() override = 0;

  /**
   * @brief Called when the node is halted
   */
  void onHalted() override = 0;

 protected:
  ros::NodeHandle& node_;  // ROS node handle for communication
};

/**
 * @brief Registers a stateful action node with the behavior tree factory
 *
 * This template function creates and registers a builder for derived node types,
 * allowing them to be instantiated by the behavior tree factory.
 *
 * @param factory The behavior tree factory to register with
 * @param registration_ID Unique identifier for the node type
 * @param node_handle ROS node handle for communication
 */
template <class DerivedT>
static void RegisterStatefulActionNodeROS(BT::BehaviorTreeFactory& factory, const std::string& registration_ID, ros::NodeHandle& node_handle) {
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<DerivedT>(node_handle, name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = StatefulActionNodeROS::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
};

/**
 * @brief Planning mode enumeration
 * Defines different modes of operation for the local planner
 */
enum PLAN : std::uint8_t {
  SINGLE_BAND,  //!< Single elastic band optimization
  DUAL_BAND,    //!< Dual elastic band optimization for robot and human
  VELOBS,       //!< Velocity obstacles-based planning
  BACKOFF,      //!< Backoff behavior when stuck
  PASSTHROUGH   //!< PassThrough Mode at passages
};

/**
 * @brief Prediction mode enumeration
 * Defines different strategies for predicting human motion
 */
enum PREDICTION : std::uint8_t {
  CONST_VEL,  //!< Constant velocity prediction
  BEHIND,     //!< Predicting the goal as behind the robot
  PREDICT,    //!< Use internal goal prediction scheme
  EXTERNAL    //!< External prediction provided the human goals
};

/**
 * @brief Structure combining planning and prediction modes
 * Used to configure the behavior of the local planner
 */
// 'Using' leads to linkage errors
typedef struct {
  PLAN plan;           //!< Selected planning mode
  PREDICTION predict;  //!< Selected prediction strategy
} ModeInfo;

}  // namespace hateb_local_planner

#endif  // BT_CORE_HH_
