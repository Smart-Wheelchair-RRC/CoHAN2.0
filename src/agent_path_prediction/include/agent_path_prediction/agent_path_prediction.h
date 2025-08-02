/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2020-2025 LAAS-CNRS
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

#ifndef AGENT_PATH_PREDICTION_H_
#define AGENT_PATH_PREDICTION_H_

#include <agent_path_prediction/AgentGoal.h>
#include <agent_path_prediction/AgentPathPredictionConfig.h>
#include <agent_path_prediction/AgentPose.h>
#include <agent_path_prediction/AgentPosePredict.h>
#include <agent_path_prediction/PredictedGoal.h>
#include <agent_path_prediction/PredictedGoals.h>
#include <agent_path_prediction/predict_goal.h>
#include <cohan_msgs/AgentPathArray.h>
#include <cohan_msgs/TrackedAgents.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <string>

//  TF2
// #include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Some fixed parameters
#define ANG_VEL_EPS 0.001
#define MAX_AGENT_MARKERS 1000
#define MIN_MARKER_LIFETIME 1.0
#define MINIMUM_COVARIANCE_MARKERS 0.1
#define RECALC_DIST 0.5
#define DEFAULT_AGENT_PART cohan_msgs::TrackedSegmentType::TORSO
#define NODE_NAME "agent_path_prediction"

namespace agents {
class AgentPathPrediction {
 public:
  /**
   * @brief Default constructor for AgentPathPrediction class */
  AgentPathPrediction() = default;

  /**
   * @brief Default destructor for AgentPathPrediction class */
  ~AgentPathPrediction() = default;

  /**
   * @brief Initializes the AgentPathPrediction node and sets up ROS communication */
  void initialize();

  /**
   * @brief Sets parameters for velocity obstacle-based prediction
   * @param velobs_mul Velocity obstacle multiplier
   * @param velobs_min_rad Minimum radius for velocity obstacle
   * @param velobs_max_rad Maximum radius for velocity obstacle
   * @param velobs_max_rad_time Time for maximum radius  velocity obstacle calculation
   * @param velobs_use_ang Whether to use angular velocity in predictions
   */
  void setParams(double velobs_mul, double velobs_min_rad, double velobs_max_rad, double velobs_max_rad_time, bool velobs_use_ang);

 private:
  // Structs
  /**
   * @brief Structure to store agent path and velocity information */
  struct AgentPathVel {
    uint64_t id;                                   // Agent ID
    nav_msgs::Path path;                           // Predicted path for the agent
    geometry_msgs::TwistWithCovariance start_vel;  // Initial velocity of the agent
  };

  /**
   * @brief Structure to store agent's initial pose and velocity
   */
  struct AgentStartPoseVel {
    uint64_t id;                             // Agent ID
    geometry_msgs::PoseStamped pose;         // Initial pose of the agent
    geometry_msgs::TwistWithCovariance vel;  // Initial velocity of the agent
  };

  // ROS Publishers and Subscribers
  ros::Publisher predicted_agents_pub_;  // Publisher for predicted agent paths
  ros::Publisher front_pose_pub_;        // Publisher for front pose information
  ros::Subscriber tracked_agents_sub_;   // Subscriber for tracked agents information
  ros::Subscriber external_paths_sub_;   // Subscriber for external path information
  ros::Subscriber predicted_goal_sub_;   // Subscriber for predicted goals

  // ROS Services
  ros::ServiceServer predict_agents_server_;             // Server for agent prediction service
  ros::ServiceServer set_goal_srv_;                      // Server for setting agent goals
  ros::ServiceServer reset_prediction_services_server_;  // Server for resetting predictions
  ros::ServiceClient get_plan_client_;                   // Client for getting navigation plans

  // Transform listener
  tf2_ros::Buffer tf_;                                       // TF2 buffer for coordinate transformations
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  // TF2 transform listener for coordinate transformations

  // subscriber callbacks
  /**
   * @brief Callback for tracked agents updates
   * @param tracked_agents The tracked agents message containing current agent states
   */
  void trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents);

  /**
   * @brief Callback for external path updates
   * @param external_paths Array of external paths for agents
   */
  void externalPathsCB(const cohan_msgs::AgentPathArray::ConstPtr &external_paths);

  /**
   * @brief Callback for predicted goal updates
   * @param predicted_goal The predicted goals message
   */
  void predictedGoalCB(const agent_path_prediction::PredictedGoals::ConstPtr &predicted_goal);

  // Service callbacks
  /**
   * @brief Service to predict agent paths using default prediction method
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @return True if prediction successful, false otherwise
   */
  bool predictAgents(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);

  /**
   * @brief Service to predict agent paths using velocity obstacle method
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @return True if prediction successful, false otherwise
   */
  bool predictAgentsVelObs(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) const;

  /**
   * @brief Service to predict agent paths using external path information
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @return True if prediction successful, false otherwise
   */
  bool predictAgentsExternal(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);

  /**
   * @brief Service to predict paths for agents assuming their goal is behind the robot
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @return True if prediction successful, false otherwise
   */
  bool predictAgentsBehind(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);

  /**
   * @brief Service to predict agent paths based on predicted goals
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @return True if prediction successful, false otherwise
   */
  bool predictAgentsGoal(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);

  /**
   * @brief Service to predict agent paths from existing path data. This method is called internally at the end of different prediction mechanisms.
   * @param req Service request containing agent data
   * @param res Service response with predicted paths
   * @param path_vels Vector of agent paths and velocities
   * @return True if prediction successful, false otherwise
   */
  bool predictAgentsFromPaths(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res, const std::vector<AgentPathVel> &path_vels);

  /**
   * @brief Service to set goals for agents
   * @param req Service request containing goal data
   * @param res Service response
   * @return True if goal setting successful, false otherwise
   */
  bool setGoal(agent_path_prediction::AgentGoal::Request &req, agent_path_prediction::AgentGoal::Response &res);

  /**
   * @brief Service to reset all prediction services
   * @param req Empty service request
   * @param res Empty service response
   * @return True if reset successful, false otherwise
   */
  bool resetPredictionSrvs(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  // dynamic reconfigure variables
  dynamic_reconfigure::Server<agent_path_prediction::AgentPathPredictionConfig> *dsrv_;
  void reconfigureCB(agent_path_prediction::AgentPathPredictionConfig &config, uint32_t level);

  // Internal Methods
  /**
   * @brief Loads ROS parameters from the node handle
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh);

  /**
   * @brief Creates a fixed path from a start pose. It adds constant velocity based prediction for 0.5m infront of the human.
   * @param start_pose The starting pose for the path (human's current position)
   * @return The generated fixed path
   */
  static nav_msgs::Path setFixedPath(const geometry_msgs::PoseStamped &start_pose);

  /**
   * @brief Prunes a path starting from a given index based on pose
   * @param begin_index Starting index for pruning
   * @param pose Reference pose for pruning
   * @param path Vector of poses to prune
   * @return Index of the given pose in the vector
   */
  static size_t prunePath(size_t begin_index, const geometry_msgs::Pose &pose, const std::vector<geometry_msgs::PoseWithCovarianceStamped> &path);

  /**
   * @brief Transforms pose and twist of agent to a target frame
   * @param tracked_agents Tracked agents data
   * @param agent_id ID of the agent to transform
   * @param to_frame Target frame for transformation
   * @param pose Output transformed pose
   * @param twist Output transformed twist
   * @return True if transformation successful, false otherwise
   */
  bool transformPoseTwist(const cohan_msgs::TrackedAgents &tracked_agents, const uint64_t &agent_id, const std::string &to_frame, geometry_msgs::PoseStamped &pose,
                          geometry_msgs::TwistStamped &twist) const;

  /**
   * @brief Calculates Euclidean distance between agent and robot poses
   * @param agent Agent pose
   * @param robot Robot pose
   * @return Distance between agent and robot
   */
  static double checkdist(geometry_msgs::Pose agent, geometry_msgs::Pose robot) { return std::hypot(agent.position.x - robot.position.x, agent.position.y - robot.position.y); }

  geometry_msgs::TwistStamped transformTwist(const geometry_msgs::TwistStamped &twist_in, const std::string &target_frame) const;

  // Properties
  cohan_msgs::TrackedAgents tracked_agents_;                                 //!< Current state of tracked agents in the environment
  cohan_msgs::AgentPathArray::ConstPtr external_paths_;                      //!< External paths provided for agents
  agent_path_prediction::PredictedGoals predicted_goals_;                    //!< Collection of predicted goals for agents
  std::vector<agent_path_prediction::AgentPose> external_goals_;             //!< Vector storing external goals for agents
  std::vector<AgentPathVel> path_vels_;                                      //!< Vector storing path and velocity information for agents
  std::vector<int> path_vels_pos_;                                           //!< Vector storing positions in the path velocity array
  std::vector<agent_path_prediction::PredictedPoses> last_predicted_poses_;  //!< Vector storing the last predicted poses for agents
  std::map<uint64_t, size_t> last_prune_indices_;                            //!< Map storing last pruning indices for each agent
  std::map<uint64_t, int> last_markers_size_map_;                            //!< Map storing last marker sizes for each agent
  visualization_msgs::MarkerArray predicted_agents_markers_;                 //!< Visualization markers for predicted agent paths
  std::string tracked_agents_sub_topic_, external_paths_sub_topic_, predict_service_name_, predicted_agents_markers_pub_topic_, get_plan_srv_name_,
      predicted_goal_topic_;                                                   //!< ROS topic names for subscribers and publishers
  std::string robot_frame_id_, map_frame_id_;                                  //!< Frame IDs for coordinate transformations
  double velobs_mul_, velobs_min_rad_, velobs_max_rad_, velobs_max_rad_time_;  //!< Velocity obstacle parameters
  double agent_dist_behind_robot_, agent_angle_behind_robot_;                  //!< Parameters for agents behind robot detection
  bool velobs_use_ang_, check_path_;                                           //!< Flags for velocity obstacles and path checking
  bool publish_markers_, showing_markers_,                                     //!< Flags for marker visualization
      got_new_agent_paths_, got_external_goal_;                                //!< Flags for path updates
  int default_agent_part_;                                                     //!< Default body part to track for agents
  geometry_msgs::Transform behind_pose_;                                       //!< Transform for behind pose calculation
  std::string ns_;                                                             //!< Namespace for the node
};
}  // namespace agents

#endif  // AGENT_PATH_PREDICTION_H_
