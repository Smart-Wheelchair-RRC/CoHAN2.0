#include <hateb_local_planner/behavior_tree/condition/is_goal_updated.h>

#include "hateb_local_planner/behavior_tree/bt_core.h"

namespace hateb_local_planner {

IsGoalUpdated::IsGoalUpdated(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // set the node name
  name_ = condition_name;
}

IsGoalUpdated::~IsGoalUpdated() {
  // ROS_INFO in destructor
  ROS_INFO("Shutting down the isGoalUpdated BT Node");
}

BT::NodeStatus IsGoalUpdated::tick() {
  bool updated = false;
  bool recovery = true;
  // Read the values from black board
  getInput("goal_update", updated);
  getInput("recovery", recovery);

  // Goal updated case
  // (Note: This will not be called when a manual goal is given during backoff recovery. Is it an issue?)
  if (updated && !recovery) {
    BT_INFO(name_, "Goal updated, restarting the tree!")
    return BT::NodeStatus::SUCCESS;
  }

  BT_INFO(name_, "Goal in progress.")
  return BT::NodeStatus::FAILURE;
}

};  // namespace hateb_local_planner