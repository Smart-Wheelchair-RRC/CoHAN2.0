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

#include <hateb_local_planner/mode_switch.h>

#include <string>

#define AGENTS_INFO_SUB "/move_base/agentsInfo"
#define GOAL_SUB "/move_base/goal"
#define RESULT_SUB "/move_base/result"
#define PASSAGE_SUB "/map_scanner/passage"

namespace hateb_local_planner {
ModeSwitch::ModeSwitch() {
  name_ = "ModeSwitch";
  initialized_ = false;
};

void ModeSwitch::initialize(ros::NodeHandle& nh, std::string& xml_path, std::shared_ptr<agents::Agents>& agents_ptr, std::shared_ptr<Backoff>& backoff_ptr) {
  if (!initialized_) {
    // Initialize the ROS components
    // TODO(sphanit): Check if you need to make them configurable
    nh_ = nh;

    // Get the namespace from the parameter server (different from the cfg server)
    if (!ros::param::get("~ns", ns_)) {
      ns_ = std::string("");
    }

    // Map the subcsriptions properly
    agents_info_sub_topic_ = std::string(AGENTS_INFO_SUB);
    goal_sub_topic_ = std::string(GOAL_SUB);
    result_sub_topic_ = std::string(RESULT_SUB);
    passage_sub_topic_ = std::string(PASSAGE_SUB);
    if (!ns_.empty()) {
      agents_info_sub_topic_ = "/" + ns_ + agents_info_sub_topic_;
      goal_sub_topic_ = "/" + ns_ + goal_sub_topic_;
      result_sub_topic_ = "/" + ns_ + result_sub_topic_;
      passage_sub_topic_ = "/" + ns_ + passage_sub_topic_;
    }

    agents_info_sub_ = nh_.subscribe(agents_info_sub_topic_, 1, &ModeSwitch::agentsInfoCB, this);
    goal_sub_ = nh_.subscribe(goal_sub_topic_, 1, &ModeSwitch::goalMoveBaseCB, this);
    result_sub_ = nh_.subscribe(result_sub_topic_, 1, &ModeSwitch::resultMoveBaseCB, this);
    passage_detect_sub_ = nh_.subscribe(passage_sub_topic_, 1, &ModeSwitch::passageCB, this);
    planning_mode_pub_ = nh_.advertise<hateb_local_planner::PlanningMode>("planning_mode", 10);

    // Initialize the parameters
    goal_reached_ = true;
    goal_update_ = false;
    backoff_ptr_ = backoff_ptr;

    // Register the BT nodes
    registerNodes();

    // Build the Behavior Tree from the XML
    if (xml_path == "") {
      BT_ERROR("ModeSwitch", "Please provide the correct xml to create the tree!")
      exit(0);
    }
    bhv_tree_ = bhv_factory_.createTreeFromFile(xml_path);
    int8_t psg_type = cohan_msgs::PassageType::OPEN;

    // Set the initial Blackboard entries
    ModeInfo init_mode;
    init_mode.plan = PLAN::SINGLE_BAND;
    init_mode.predict = PREDICTION::CONST_VEL;
    bhv_tree_.rootBlackboard()->set("planning_mode", init_mode);
    bhv_tree_.rootBlackboard()->set("goal_update", false);
    bhv_tree_.rootBlackboard()->set("backoff_ptr", backoff_ptr);
    bhv_tree_.rootBlackboard()->set("agents_ptr", agents_ptr);
    bhv_tree_.rootBlackboard()->set("passage_type", psg_type);
    bhv_tree_.rootBlackboard()->set("reset", false);
    bhv_tree_.rootBlackboard()->set("recovery", false);

    BT_INFO(name_, "Behavior Tree initialized.")
    initialized_ = true;
  } else {
    BT_WARN(name_, "The tree is already initialized!")
  }
}

void ModeSwitch::passageCB(const cohan_msgs::PassageType& passage_msg) {
  // Set the passage type on the blackboard
  bhv_tree_.rootBlackboard()->set("passage_type", passage_msg.type);
}

void ModeSwitch::agentsInfoCB(const agent_path_prediction::AgentsInfo& info_msg) {
  agents_info_ = info_msg;

  // Set the agents_info on the blackboard
  bhv_tree_.rootBlackboard()->set("agents_info", agents_info_);
}

void ModeSwitch::goalMoveBaseCB(const move_base_msgs::MoveBaseActionGoal& goal_msg) {
  // Set the goal status
  BT_INFO(name_, "Goal is set!")
  if (!goal_reached_) {
    bhv_tree_.rootBlackboard()->set("goal_update", true);
    goal_update_ = true;
  }
  goal_ = goal_msg.goal.target_pose;
  bhv_tree_.rootBlackboard()->set("nav_goal", goal_);
  goal_reached_ = false;
}

void ModeSwitch::resultMoveBaseCB(const move_base_msgs::MoveBaseActionResult& result_msg) {
  // Set the goal status
  if (static_cast<int>(result_msg.status.status) == 3) {
    goal_reached_ = true;
  }
}

BT::NodeStatus ModeSwitch::tickBT() {
  // Tick the tree from the start
  auto status = bhv_tree_.tickOnce();
  updateMode();
  if (goal_update_) {
    bhv_tree_.rootBlackboard()->set("goal_update", false);
    goal_update_ = false;
  }
  return status;
}

void ModeSwitch::updateMode(int duration) {
  std::scoped_lock lock(pub_mutex_);

  // Get the PlanningMode msg from the blackboard
  mode_info_ = bhv_tree_.rootBlackboard()->get<ModeInfo>("planning_mode");

  BT_INFO(name_, (int)mode_info_.plan);
  BT_INFO(name_, (int)mode_info_.predict);

  plan_mode_.plan_mode = mode_info_.plan;
  plan_mode_.predict_mode = mode_info_.predict;
  plan_mode_.moving_humans = agents_info_.moving;
  plan_mode_.still_humans = agents_info_.still;

  // TODO(sphanit): Make the duration configurable. will this be of any advantage?
  // Publish the mode on the give ROS Topic
  if (duration == 0) {
    planning_mode_pub_.publish(plan_mode_);
  } else {
    auto start = ros::Time::now();
    auto end = ros::Time::now();
    while (((end - start).toSec() != duration)) {
      end = ros::Time::now();
      planning_mode_pub_.publish(plan_mode_);
    }
  }
}

hateb_local_planner::PlanningMode ModeSwitch::tickAndGetMode() {
  // Tick the tree once and return the updated planning mode
  tickBT();
  return plan_mode_;
}

void ModeSwitch::resetBT() {
  // Halt the tree and set goal reached to true
  goal_reached_ = true;
  bhv_tree_.haltTree();
  // printTreeStatus(bhv_tree_.rootNode()); //<! Use this for debugging Tree status
}

void ModeSwitch::registerNodes() {
  // The only node that handles ROS connections is the "setMode"
  RegisterStatefulActionNodeROS<hateb_local_planner::SetMode>(bhv_factory_, "setMode", nh_);
  // Register all other nodes needed for the behavior tree
  bhv_factory_.registerNodeType<hateb_local_planner::IsGoalReached>("goalCheck");
  bhv_factory_.registerNodeType<hateb_local_planner::IsGoalUpdated>("isGoalUpdated");
  bhv_factory_.registerNodeType<hateb_local_planner::SingleBandExitCondition>("singleBandExitCond");
  bhv_factory_.registerNodeType<hateb_local_planner::DualBandExitCondition>("dualBandExitCond");
  bhv_factory_.registerNodeType<hateb_local_planner::VelObsExitCondition>("velobsExitCond");
  bhv_factory_.registerNodeType<hateb_local_planner::BackoffExitCondition>("backoffExitCond");
  bhv_factory_.registerNodeType<hateb_local_planner::PassThroughCondition>("passThroughCond");
}

};  // namespace hateb_local_planner
