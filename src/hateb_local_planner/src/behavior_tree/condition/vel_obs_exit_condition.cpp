#include <hateb_local_planner/behavior_tree/condition/vel_obs_exit_condition.h>

namespace hateb_local_planner {

VelObsExitCondition::VelObsExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // set the node name and initialize properties
  name_ = condition_name;
  nearest_human_id_ = -1;
  t_stuck_ = 0;
}

VelObsExitCondition::~VelObsExitCondition() {
  // ROS_INFO in destructor
  ROS_INFO("Shutting downd the VelObsExitCondition BT Node");
}

BT::NodeStatus VelObsExitCondition::tick() {
  // As the robot is stuck, increament t_stuck everytime this node is ticked
  t_stuck_++;
  if (agents_ptr_ == nullptr) {
    // Get the agents pointer from blackboard
    getInput("agents_ptr", agents_ptr_);
  }

  // Check if the human as stopped too (while the robot is stuck)
  if (hasHumanStopped() && t_stuck_ >= 20) {  // TODO(unknown): Remove the magic number 20
    t_stuck_ = 0;
    BT_INFO(name_, "Both human and robot are stuck. Exiting VelObs!")
    // Lock the agents pointer before updating
    std::scoped_lock lock(agents_mutex_);
    // Set the human state to BLOCKED
    agents_ptr_->setState(agents::AgentState::BLOCKED, nearest_human_id_);
    // Update the blackboard
    // setOutput("stuck_agent", nearest_human_id_); // Not needed
    return BT::NodeStatus::SUCCESS;
  }

  BT_INFO(name_, "in VelObs")
  return BT::NodeStatus::FAILURE;
}

bool VelObsExitCondition::hasHumanStopped() {
  getInput("agents_info", agents_info_);

  if (!agents_info_.humans.empty()) {
    // Store human id of the closest human
    if (nearest_human_id_ != agents_info_.visible[0]) {
      nearest_human_id_ = agents_info_.visible[0];
    }

    BT_INFO(name_, (int)agents_info_.humans[0].state);

    // If the human has either stopped moving or already blocked, exit the mode and start backoff recovery
    if (agents_info_.humans[0].state == agents::AgentState::STOPPED || agents_info_.humans[0].state == agents::AgentState::BLOCKED) {
      return true;
    }
  }

  return false;
}

};  // namespace hateb_local_planner