#include <hateb_local_planner/behavior_tree/condition/passthrough_condition.h>

namespace hateb_local_planner {

PassThroughCondition::PassThroughCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // Initialize the node
  name_ = condition_name;
}

PassThroughCondition::~PassThroughCondition() {
  // ROS_INFO in destructor
  ROS_INFO("Shutting downd the PassThroughCondition BT Node");
}

BT::NodeStatus PassThroughCondition::tick() {
  getInput("passage_type", psg_type_);

  // PassThrough is activated when a DOOR or PILLAR is detected
  if (psg_type_ == cohan_msgs::PassageType::DOOR || psg_type_ == cohan_msgs::PassageType::PILLAR) {
    BT_INFO(name_, "Passage detected!")
    return BT::NodeStatus::SUCCESS;
  }

  BT_INFO(name_, "No Passage Detected!")
  return BT::NodeStatus::FAILURE;
}
};  // namespace hateb_local_planner