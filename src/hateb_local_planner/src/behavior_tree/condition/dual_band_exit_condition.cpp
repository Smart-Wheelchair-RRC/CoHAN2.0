#include <hateb_local_planner/behavior_tree/condition/dual_band_exit_condition.h>

namespace hateb_local_planner {

DualBandExitCondition::DualBandExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  name_ = condition_name;
  // Initialize the variables
  dist_threshold_ = 999;
  goal_dist_ = 999;
}

DualBandExitCondition::~DualBandExitCondition() {
  // ROS_INFO in destructor
  ROS_INFO("Shutting downd the DualBandExitCondition BT Node");
}

BT::NodeStatus DualBandExitCondition::tick() {
  // Get the data from blackboard
  getInput("agents_info", agents_info_);
  getInput("dist_threshold", dist_threshold_);  // set in the xml of bt tree

  // Exit the mode if the robot is stuck in any of the two cases
  if (isRobotStuck()) {
    if (!agents_info_.humans.empty()) {
      auto human = agents_info_.humans[0];
      // Only exit the band if the human is under the dist_threshold
      if (human.dist <= dist_threshold_ && (human.state == agents::AgentState::STOPPED)) {
        BT_INFO(name_, "The robot is stuck and human is still, Exiting Dual Band!")
        return BT::NodeStatus::SUCCESS;
      }
      if (human.dist <= dist_threshold_) {
        BT_INFO(name_, "The robot is stuck and human is moving, Exiting Dual Band!")
        return BT::NodeStatus::SUCCESS;
      }
    }
    // Exit even when no human is detected
    else {
      BT_INFO(name_, "The robot is stuck, Exiting Dual Band!")
      return BT::NodeStatus::SUCCESS;
    }
  }

  BT_INFO(name_, "in Dual Band")
  return BT::NodeStatus::FAILURE;
}

bool DualBandExitCondition::isRobotStuck() {
  // Get the inputs from blackboard
  getInput("agents_info", agents_info_);
  getInput("nav_goal", goal_);

  double dx = goal_.pose.position.x - agents_info_.robot_pose.x;
  double dy = goal_.pose.position.y - agents_info_.robot_pose.y;

  // Check if distance to goal is constantly decreasing or not
  if (fabs(goal_dist_ - std::hypot(dx, dy)) > DIST_EPS) {
    stopped_time_ = ros::Time::now();
    goal_dist_ = std::hypot(dx, dy);
  }

  // If the goal_dist is not decreasing for over 2.0 sec, exit the mode
  if ((ros::Time::now() - stopped_time_).toSec() >= 2.0) {  // TODO(sphanit): Remove the magic number 2.0s here
    if (goal_dist_ - std::hypot(dx, dy) < 2 * DIST_EPS) {
      return true;
    }
  }

  // Otherwise the robot is moving well
  BT_INFO(name_, "Robot is moving!")
  return false;
}

};  // namespace hateb_local_planner