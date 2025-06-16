#include <hateb_local_planner/behavior_tree/condition/single_band_exit_condition.h>

#include "hateb_local_planner/behavior_tree/bt_core.h"

namespace hateb_local_planner {

SingleBandExitCondition::SingleBandExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // Initialize the node
  name_ = condition_name;
  dist_max_ = 999;
}

SingleBandExitCondition::~SingleBandExitCondition() {
  // ROS_INFO in destructor
  ROS_INFO("Shutting downd the SingleBandExitCondition BT Node");
}

BT::NodeStatus SingleBandExitCondition::tick() {
  // Read the values from blackboard
  getInput("agents_info", agents_info_);
  getInput("dist_max", dist_max_);  // Set in bt tree xml

  // If no humans, single band mode always
  if (!agents_info_.humans.empty()) {
    // Get the distance of the nearest human from the robot
    auto dist = agents_info_.humans[0].dist;
    // If any human inside planning radius is moving, switch to dual band mode
    if (dist < dist_max_) {
      for (auto& human : agents_info_.humans) {
        if (human.state != agents::AgentState::STATIC && human.state != agents::AgentState::STOPPED) {  //<! the STOPPED condition to reset things properly
          BT_INFO(name_, "Exiting Single Band!")
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
  }

  BT_INFO(name_, "in Single Band")
  return BT::NodeStatus::FAILURE;
}

};  // namespace hateb_local_planner