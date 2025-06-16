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
 *  Author: Phani Teja Singamaneni
 *********************************************************************/

#include <agent_path_prediction/PredictedGoal.h>
#include <agent_path_prediction/PredictedGoals.h>
#include <agent_path_prediction/predict_goal.h>
#include <cohan_msgs/TrackedAgent.h>
#include <cohan_msgs/TrackedAgents.h>
#include <cohan_msgs/TrackedSegmentType.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace agents {
/**
 * @brief ROS wrapper for the Bayesian goal prediction system
 */
class PredictGoalROS {
 public:
  /**
   *@brief Default constructor, initializes ROS communication
   */
  PredictGoalROS();

  /**
   * @brief Default destructor
   */
  ~PredictGoalROS() = default;

 private:
  /**
   * @brief Callback for tracked agents updates. This method processes incoming tracked agents data and updates the internal state.
   * @param msg Message containing tracked agents data
   */
  void trackedAgentsCB(const cohan_msgs::TrackedAgents::ConstPtr& msg);

  /**
   * @brief Load goal positions from a YAML file. This method reads goal positions from the specified YAML file and populates the goals map.
   * @param file Path to the YAML file containing goals
   * @return True if goals were loaded successfully, false otherwise
   */
  bool loadGoals(const std::string& file);

  // Internal Methods
  /**
   * @brief Loads ROS parameters from the node handle
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);

  // ROS
  ros::Subscriber agents_sub_;  //!< Subscriber for tracked agents updates
  ros::Publisher goal_pub_;     //!< Publisher for predicted goals

  // Core components
  agents::BayesianGoalPrediction predictor_;  //!< Instance of the Bayesian goal prediction algorithm

  // Data storage
  std::map<std::string, Eigen::Vector2d> goals_;    //!< Map of goal names to their positions
  std::map<int, std::string> agent_goal_predicts_;  //!< Map of agent IDs to their predicted goals

  // Configuration
  int window_size_;  //!< Size of the sliding window used for predictions

  std::string tracked_agents_sub_topic_, predicted_goal_topic_;  //!< ROS topic names for subscribers and publishers
  std::string ns_;                                               //!< Namespace for the node
  std::string goals_file_;                                       //!< File name of the env goals
};
}  // namespace agents