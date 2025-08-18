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

  /**
   * @brief Look up twist information between frames with default reference point
   *
   * Simplified version that uses the origin of tracking_frame as the reference point
   * and observation_frame as the reference frame.
   * @param tracking_frame The frame being tracked
   * @param observation_frame The frame from which we're observing
   * @param time The time at which to get the twist
   * @param averaging_interval Time interval over which to average the twist
   * @param[out] twist The resulting twist message
   */
  void lookupTwist(const std::string &tracking_frame, const std::string &observation_frame, const ros::Time &time, const ros::Duration &averaging_interval, geometry_msgs::Twist &twist) const {
    // ref point is origin of tracking_frame, ref_frame = obs_frame
    lookupTwist(tracking_frame, observation_frame, observation_frame, tf2::Vector3(0, 0, 0), tracking_frame, time, averaging_interval, twist);
  }

  /**
   * @brief Look up twist information between frames with custom reference point
   *
   * This method computes the twist (linear and angular velocity) of one frame
   * relative to another, allowing specification of a custom reference point and frame.
   * It performs the necessary coordinate transformations and averages the motion
   * over the specified time interval.
   *
   * @param tracking_frame The frame being tracked (whose motion we want to determine)
   * @param observation_frame The frame from which we're observing
   * @param reference_frame The frame in which the twist should be expressed
   * @param reference_point The point about which the twist should be computed
   * @param reference_point_frame The frame in which the reference point is expressed
   * @param time The time at which to get the twist
   * @param averaging_interval Time interval over which to average the twist
   * @param[out] twist The resulting twist message containing linear and angular velocities
   */
  void lookupTwist(const std::string &tracking_frame, const std::string &observation_frame, const std::string &reference_frame, const tf2::Vector3 &reference_point,
                   const std::string &reference_point_frame, const ros::Time &time, const ros::Duration &averaging_interval, geometry_msgs::Twist &twist) const {
    ros::Time latest_time;
    ros::Time target_time;

    tf2::CompactFrameID target_id = tf_._lookupFrameNumber(strip_leading_slash(tracking_frame));
    tf2::CompactFrameID source_id = tf_._lookupFrameNumber(strip_leading_slash(observation_frame));
    tf_._getLatestCommonTime(source_id, target_id, latest_time, nullptr);

    if (ros::Time() == time) {
      target_time = latest_time;
    } else {
      target_time = time;
    }

    ros::Time end_time = std::min(target_time + averaging_interval * 0.5, latest_time);

    ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
    ros::Duration corrected_averaging_interval = end_time - start_time;                                                // correct for the possiblity that start time was
                                                                                                                       // truncated above.
    geometry_msgs::TransformStamped start_msg;
    geometry_msgs::TransformStamped end_msg;
    start_msg = tf_.lookupTransform(observation_frame, tracking_frame, start_time);
    end_msg = tf_.lookupTransform(observation_frame, tracking_frame, end_time);

    tf2::Stamped<tf2::Transform> start;
    tf2::Stamped<tf2::Transform> end;
    tf2::fromMsg(start_msg, start);
    tf2::fromMsg(end_msg, end);

    tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
    tf2::Quaternion quat_temp;
    temp.getRotation(quat_temp);
    tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
    double ang = quat_temp.getAngle();

    double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
    double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
    double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();

    tf2::Vector3 twist_vel((delta_x) / corrected_averaging_interval.toSec(), (delta_y) / corrected_averaging_interval.toSec(), (delta_z) / corrected_averaging_interval.toSec());
    tf2::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());

    // This is a twist w/ reference frame in observation_frame  and reference
    // point is in the tracking_frame at the origin (at start_time)

    // correct for the position of the reference frame
    tf2::Stamped<tf2::Transform> inverse;
    tf2::fromMsg(tf_.lookupTransform(reference_frame, tracking_frame, target_time), inverse);
    tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
    tf2::Vector3 out_vel = inverse.getBasis() * twist_vel + inverse.getOrigin().cross(out_rot);

    // Rereference the twist about a new reference point
    // Start by computing the original reference point in the reference frame:
    tf2::Stamped<tf2::Vector3> rp_orig(tf2::Vector3(0, 0, 0), target_time, tracking_frame);
    geometry_msgs::TransformStamped reference_frame_trans;
    tf2::fromMsg(tf_.lookupTransform(reference_frame, rp_orig.frame_id_, rp_orig.stamp_), reference_frame_trans);

    geometry_msgs::PointStamped rp_orig_msg;
    tf2::toMsg(rp_orig, rp_orig_msg);
    tf2::doTransform(rp_orig_msg, rp_orig_msg, reference_frame_trans);

    // convert the requrested reference point into the right frame
    tf2::Stamped<tf2::Vector3> rp_desired(reference_point, target_time, reference_point_frame);
    geometry_msgs::PointStamped rp_desired_msg;
    tf2::toMsg(rp_desired, rp_desired_msg);
    tf2::doTransform(rp_desired_msg, rp_desired_msg, reference_frame_trans);
    // compute the delta
    tf2::Vector3 delta = rp_desired - rp_orig;
    // Correct for the change in reference point
    out_vel = out_vel + out_rot * delta;
    // out_rot unchanged

    twist.linear.x = out_vel.x();
    twist.linear.y = out_vel.y();
    twist.linear.z = out_vel.z();
    twist.angular.x = out_rot.x();
    twist.angular.y = out_rot.y();
    twist.angular.z = out_rot.z();
  }

  std::string strip_leading_slash(const std::string &frame_id) const {
    if (!frame_id.empty() && frame_id[0] == '/') {
      return frame_id.substr(1);
    }
    return frame_id;
  }

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
