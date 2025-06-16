#include <hateb_local_planner/behavior_tree/action/set_mode.h>

namespace hateb_local_planner {

SetMode::SetMode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfig& config) : StatefulActionNodeROS(nh, name, config) {
  // set the name of the node
  name_ = name;
  // planning_mode_pub_ = node_.advertise<hateb_local_planner::PlanningMode>("planning_mode", 10);
}

SetMode::~SetMode() {
  // Set the ROS_INFO while shutting down the node
  ROS_INFO("Shutting down the setMode BT Node");
}

BT::NodeStatus SetMode::onStart() {
  getInput("plan_type", plan_type_);
  getInput("predict_type", predict_type_);

  // Update the planning mode on the blackboard based on the type
  if (plan_type_ == "single") {
    p_msg_.plan = PLAN::SINGLE_BAND;
  }

  else if (plan_type_ == "dual") {
    p_msg_.plan = PLAN::DUAL_BAND;
  }

  else if (plan_type_ == "velobs") {
    p_msg_.plan = PLAN::VELOBS;
  }

  else if (plan_type_ == "backoff") {
    p_msg_.plan = PLAN::BACKOFF;
  }

  else if (plan_type_ == "passthrough") {
    p_msg_.plan = PLAN::PASSTHROUGH;
  }

  if (predict_type_ == "const_vel") {
    p_msg_.predict = PREDICTION::CONST_VEL;
  }

  else if (predict_type_ == "behind") {
    p_msg_.predict = PREDICTION::BEHIND;
  }

  else if (predict_type_ == "predict") {
    p_msg_.predict = PREDICTION::PREDICT;
  }

  else if (predict_type_ == "external") {
    p_msg_.predict = PREDICTION::EXTERNAL;
  }

  // Port Remapping has to be called before using setOutput (notice name change)
  setOutput("mode", p_msg_);

  BT_INFO(name_, "onStart()")
  // Set the status to RUNNING
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMode::onRunning() {
  // Keep Running until halt
  BT_INFO(name_, "onRunning()")
  return BT::NodeStatus::RUNNING;
}

void SetMode::onHalted() {
  // Just update the info
  BT_INFO(name_, "Node is interrupted by calling onHalted()");
}

};  // namespace hateb_local_planner
