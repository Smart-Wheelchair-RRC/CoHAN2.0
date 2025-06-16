#include <agent_path_prediction/predict_goal_ros.h>

#define AGENTS_SUB_TOPIC "/tracked_agents"

namespace agents {

PredictGoalROS::PredictGoalROS() {
  // ROS setup
  ros::NodeHandle nh("~");

  // Get params
  loadRosParamFromNodeHandle(nh);

  if (goals_file_.empty()) {
    ROS_ERROR("Please provide a valid file path for goals files!");
  }

  goal_pub_ = nh.advertise<agent_path_prediction::PredictedGoals>("predicted_goal", 2, true);

  // Need to remap tracked agents subscriber properly
  if (!ns_.empty()) {
    tracked_agents_sub_topic_ = "/" + ns_ + tracked_agents_sub_topic_;
  }

  // Initialize Subscribers
  agents_sub_ = nh.subscribe(tracked_agents_sub_topic_, 10, &PredictGoalROS::trackedAgentsCB, this);

  // Load goals file
  loadGoals(goals_file_);
  predictor_.initialize(goals_, window_size_);
  ROS_INFO("Goal prediction intialized!");
}

void PredictGoalROS::trackedAgentsCB(const cohan_msgs::TrackedAgents::ConstPtr &msg) {
  bool changed = false;
  for (const auto &agent : msg->agents) {
    for (const auto &segment : agent.segments) {
      // Make sure you are getting the correct segment data
      if (segment.type == cohan_msgs::TrackedSegmentType::TORSO) {
        auto xy = Eigen::Vector2d(segment.pose.pose.position.x, segment.pose.pose.position.y);
        if (agent_goal_predicts_.find(agent.track_id) == agent_goal_predicts_.end()) {
          agent_goal_predicts_[agent.track_id] = "None";
        }
        auto goal = predictor_.predictGoal(agent.track_id, xy);

        if (agent_goal_predicts_[agent.track_id] != goal) {
          changed = true;
        }
        // Update the predicted goals
        agent_goal_predicts_[agent.track_id] = goal;
      }
    }
  }
  if (changed) {
    // Publish the new goals
    agent_path_prediction::PredictedGoals predicted_goals_msg;
    predicted_goals_msg.header.frame_id = "map";
    predicted_goals_msg.header.stamp = ros::Time::now();
    for (auto &agent_goal : agent_goal_predicts_) {
      agent_path_prediction::PredictedGoal p_goal;
      p_goal.id = agent_goal.first;
      p_goal.goal.position.x = goals_[agent_goal.second].x();
      p_goal.goal.position.y = goals_[agent_goal.second].y();
      p_goal.goal.position.z = 0.0;
      p_goal.goal.orientation.w = 1;
      predicted_goals_msg.goals.push_back(p_goal);
    }
    goal_pub_.publish(predicted_goals_msg);
  }
}

bool PredictGoalROS::loadGoals(const std::string &file) {
  goals_.clear();
  try {
    // Load goals from the YAML file
    YAML::Node config = YAML::LoadFile(file);

    window_size_ = config["window_size"].as<int>();
    const YAML::Node &goals = config["goals"];

    // Iterate through each goal
    for (const auto &goal : goals) {
      std::string name = goal["name"].as<std::string>();
      const auto &coordinates = goal["goal"];

      goals_[name] = Eigen::Vector2d(coordinates[0].as<double>(), coordinates[1].as<double>());
    }
  } catch (const YAML::Exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return false;
  }
  return false;
}

void PredictGoalROS::loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh) {
  private_nh.param("ns", ns_, std::string(""));
  private_nh.param("goals_file", goals_file_, std::string(""));
  private_nh.param("tracked_agents_sub_topic", tracked_agents_sub_topic_, std::string(AGENTS_SUB_TOPIC));
}

};  // namespace agents