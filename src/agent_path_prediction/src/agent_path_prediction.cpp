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

#include <agent_path_prediction/agent_path_prediction.h>
#include <agent_path_prediction/predict_goal_ros.h>

#include <csignal>
#include <utility>

// Reconfigurable Parameters
#define AGENTS_SUB_TOPIC "/tracked_agents"                      //!< Absolute topic name for the tracked agents
#define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"  //!< Absolute service name for the global planner
#define PREDICTED_GOAL_SUB_TOPIC "predicted_goal"               //!< Relative topic name for the predicted goal
#define EXTERNAL_PATHS_SUB_TOPIC "external_agent_paths"         //!< Relative topic name for the external agent paths
#define ROBOT_FRAME_ID "base_footprint"
#define MAP_FRAME_ID "map"

#define AGENT_DIST_BEHIND_ROBOT 0.5
#define AGENT_ANGLE_BEHIND_ROBOT 3.14

namespace agents {

void AgentPathPrediction::initialize() {
  // get private node handle
  ros::NodeHandle private_nh("~/");

  // Load params from the server
  loadRosParamFromNodeHandle(private_nh);

  // Initialize Publishers
  predicted_agents_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("predicted_agent_poses", 1);
  front_pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("front_pose", 1);

  // Initialize Service servers
  set_goal_srv_ = private_nh.advertiseService("set_agent_goal", &AgentPathPrediction::setGoal, this);
  predict_agents_server_ = private_nh.advertiseService("predict_agent_poses", &AgentPathPrediction::predictAgents, this);
  reset_prediction_services_server_ = private_nh.advertiseService("reset_prediction_services", &AgentPathPrediction::resetPredictionSrvs, this);

  // Need to remap subscriber properly
  if (!ns_.empty()) {
    tracked_agents_sub_topic_ = "/" + ns_ + tracked_agents_sub_topic_;
    get_plan_srv_name_ = "/" + ns_ + get_plan_srv_name_;
  }

  // Initialize Subscribers
  tracked_agents_sub_ = private_nh.subscribe(tracked_agents_sub_topic_, 1, &AgentPathPrediction::trackedAgentsCB, this);
  external_paths_sub_ = private_nh.subscribe(external_paths_sub_topic_, 1, &AgentPathPrediction::externalPathsCB, this);
  predicted_goal_sub_ = private_nh.subscribe(predicted_goal_topic_, 1, &AgentPathPrediction::predictedGoalCB, this);

  // Initialize Service clients
  get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name_, true);

  // Set-up dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<agent_path_prediction::AgentPathPredictionConfig>(private_nh);
  dynamic_reconfigure::Server<agent_path_prediction::AgentPathPredictionConfig>::CallbackType cb = boost::bind(&AgentPathPrediction::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // Initialize properties
  showing_markers_ = false;
  got_new_agent_paths_ = false;
  got_external_goal_ = false;

  ROS_DEBUG_NAMED(NODE_NAME, "node %s initialized", NODE_NAME);
}

void AgentPathPrediction::trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents) {
  ROS_INFO_ONCE_NAMED(NODE_NAME, "agent_path_prediction: received agents");
  tracked_agents_ = tracked_agents;
}

void AgentPathPrediction::externalPathsCB(const cohan_msgs::AgentPathArray::ConstPtr &external_paths) {
  ROS_INFO_ONCE_NAMED(NODE_NAME, "agent_path_prediction: received agent paths");
  external_paths_ = external_paths;
  got_new_agent_paths_ = true;
}

void AgentPathPrediction::predictedGoalCB(const agent_path_prediction::PredictedGoals::ConstPtr &predicted_goals) {
  ROS_INFO_ONCE_NAMED(NODE_NAME, "agent_path_prediction: received predicted goal");
  predicted_goals_ = *predicted_goals;
}

bool AgentPathPrediction::predictAgents(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) {
  boost::function<bool(agent_path_prediction::AgentPosePredict::Request & req, agent_path_prediction::AgentPosePredict::Response & res)> prediction_function;

  switch (req.type) {
    case agent_path_prediction::AgentPosePredictRequest::VELOCITY_OBSTACLE:
      prediction_function = boost::bind(&AgentPathPrediction::predictAgentsVelObs, this, _1, _2);
      break;
    case agent_path_prediction::AgentPosePredictRequest::EXTERNAL:
      prediction_function = boost::bind(&AgentPathPrediction::predictAgentsExternal, this, _1, _2);
      break;
    case agent_path_prediction::AgentPosePredictRequest::BEHIND_ROBOT:
      prediction_function = boost::bind(&AgentPathPrediction::predictAgentsBehind, this, _1, _2);
      break;
    case agent_path_prediction::AgentPosePredictRequest::PREDICTED_GOAL:
      prediction_function = boost::bind(&AgentPathPrediction::predictAgentsGoal, this, _1, _2);
      break;
    default:
      ROS_ERROR_NAMED(NODE_NAME, "%s: unkonwn prediction type %d", NODE_NAME, req.type);
  }

  if (!prediction_function.empty() && prediction_function(req, res)) {
    if (publish_markers_) {
      // create new markers
      predicted_agents_markers_.markers.clear();

      for (auto predicted_agent : res.predicted_agents_poses) {
        if (!predicted_agent.poses.empty()) {
          auto first_pose_time = predicted_agent.poses[0].header.stamp;
          int marker_id = 0;

          for (auto predicted_agent_pose : predicted_agent.poses) {
            visualization_msgs::Marker predicted_agent_marker;
            predicted_agent_marker.header.frame_id = predicted_agent_pose.header.frame_id;
            predicted_agent_marker.header.stamp = first_pose_time;
            predicted_agent_marker.id = (predicted_agent.id * MAX_AGENT_MARKERS) + marker_id++;
            predicted_agent_marker.type = visualization_msgs::Marker::CYLINDER;
            predicted_agent_marker.action = visualization_msgs::Marker::MODIFY;
            // assuming diagonal covariance matrix (with row-major order)
            predicted_agent_marker.scale.x = std::max(predicted_agent_pose.pose.covariance[0], MINIMUM_COVARIANCE_MARKERS);
            predicted_agent_marker.scale.y = std::max(predicted_agent_pose.pose.covariance[7], MINIMUM_COVARIANCE_MARKERS);
            predicted_agent_marker.scale.z = 0.01;
            predicted_agent_marker.color.a = 1.0;
            predicted_agent_marker.color.r = 0.0;
            predicted_agent_marker.color.g = 0.0;
            predicted_agent_marker.color.b = 1.0;
            predicted_agent_marker.lifetime = ros::Duration(MIN_MARKER_LIFETIME) + (predicted_agent_pose.header.stamp - first_pose_time);
            predicted_agent_marker.pose.position.x = predicted_agent_pose.pose.pose.position.x;
            predicted_agent_marker.pose.position.y = predicted_agent_pose.pose.pose.position.y;
            // time on z axis
            predicted_agent_marker.pose.position.z = (predicted_agent_pose.header.stamp - first_pose_time).toSec();
            predicted_agents_markers_.markers.push_back(predicted_agent_marker);
          }

          auto it = last_markers_size_map_.find(predicted_agent.id);
          if (it != last_markers_size_map_.end()) {
            while (it->second >= marker_id) {
              visualization_msgs::Marker delete_agent_marker;
              delete_agent_marker.id = (predicted_agent.id * MAX_AGENT_MARKERS) + marker_id++;
              delete_agent_marker.action = visualization_msgs::Marker::DELETE;
              predicted_agents_markers_.markers.push_back(delete_agent_marker);
            }
          }
          last_markers_size_map_[predicted_agent.id] = --marker_id;
        } else {
          ROS_WARN_NAMED(NODE_NAME, "no predicted poses fro agent %d", predicted_agent.id);
        }
      }

      predicted_agents_pub_.publish(predicted_agents_markers_);
      showing_markers_ = true;

      ROS_DEBUG_NAMED(NODE_NAME, "published predicted agents");
    } else {
      if (showing_markers_) {
        predicted_agents_markers_.markers.clear();
        visualization_msgs::Marker delete_agent_markers;
        delete_agent_markers.action = 3;  // visualization_msgs::Marker::DELETEALL;
        predicted_agents_markers_.markers.push_back(delete_agent_markers);
        predicted_agents_pub_.publish(predicted_agents_markers_);
        showing_markers_ = false;
      }
    }

    return true;
  }
  return false;
}

bool AgentPathPrediction::predictAgentsVelObs(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) const {
  // validate prediction time
  if (req.predict_times.empty()) {
    ROS_ERROR_NAMED(NODE_NAME, "prediction times cannot be empty");
    return false;
  }
  if (*std::min_element(req.predict_times.begin(), req.predict_times.end()) < 0.0) {
    ROS_ERROR_NAMED(NODE_NAME, "prediction time cannot be negative");
    return false;
  }

  // get local refrence of agents
  auto agents = tracked_agents_.agents;
  auto track_frame = tracked_agents_.header.frame_id;
  auto track_time = tracked_agents_.header.stamp;

  if ((ros::Time::now() - track_time).toSec() > *std::max_element(req.predict_times.begin(), req.predict_times.end())) {
    ROS_DEBUG_NAMED(NODE_NAME,
                    "agent data is older than maximum given "
                    "prediction time, predicting nothing");
    return true;
  }

  for (const auto &agent : agents) {
    if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
      continue;
    }
    for (auto segment : agent.segments) {
      if (segment.type == default_agent_part_) {
        // calculate future agent poses based on current velocity
        agent_path_prediction::PredictedPoses predicted_poses;
        predicted_poses.id = agent.track_id;

        // get linear velocity of the agent
        tf::Vector3 linear_vel(segment.twist.twist.linear.x, segment.twist.twist.linear.y, segment.twist.twist.linear.z);

        for (auto predict_time : req.predict_times) {
          // validate prediction time
          if (predict_time < 0) {
            ROS_ERROR_NAMED(NODE_NAME, "%s: prediction time cannot be negative (give %f)", NODE_NAME, predict_time);
            return false;
          }

          geometry_msgs::PoseWithCovarianceStamped predicted_pose;
          predicted_pose.header.frame_id = track_frame;
          predicted_pose.header.stamp = track_time + ros::Duration(predict_time);

          if (velobs_use_ang_ && std::abs(segment.twist.twist.angular.z) > ANG_VEL_EPS) {
            // velocity multiplier is only applied to linear velocities
            double r = (std::hypot(linear_vel[0], linear_vel[1]) * velobs_mul_) / segment.twist.twist.angular.z;
            double theta = segment.twist.twist.angular.z * predict_time;
            double crd = r * 2 * std::sin(theta / 2);
            double alpha = std::atan2(linear_vel[1], linear_vel[0]) + (theta / 2);
            predicted_pose.pose.pose.position.x = segment.pose.pose.position.x + crd * std::cos(alpha);
            predicted_pose.pose.pose.position.y = segment.pose.pose.position.y + crd * std::sin(alpha);
            predicted_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(segment.pose.pose.orientation) + theta);
          } else {
            predicted_pose.pose.pose.position.x = segment.pose.pose.position.x + linear_vel[0] * predict_time * velobs_mul_;
            predicted_pose.pose.pose.position.y = segment.pose.pose.position.y + linear_vel[1] * predict_time * velobs_mul_;
            predicted_pose.pose.pose.orientation = segment.pose.pose.orientation;
          }

          // not using velocity multiplier for covariance matrix
          double xy_vel = hypot(linear_vel[0] * predict_time, linear_vel[1] * predict_time);
          // storing only x, y covariance in diagonal matrix
          predicted_pose.pose.covariance[0] = velobs_min_rad_ + (velobs_max_rad_ - velobs_min_rad_) * (predict_time / velobs_max_rad_time_) * xy_vel;
          predicted_pose.pose.covariance[7] = predicted_pose.pose.covariance[0];
          predicted_poses.poses.push_back(predicted_pose);

          ROS_DEBUG_NAMED(NODE_NAME,
                          "%s: predected agent (%lu) segment (%d)"
                          " pose: x=%f, y=%f, theta=%f, predict-time=%f",
                          NODE_NAME, agent.track_id, segment.type, predicted_pose.pose.pose.position.x, predicted_pose.pose.pose.position.y, tf::getYaw(predicted_pose.pose.pose.orientation),
                          predict_time);
        }

        geometry_msgs::TwistStamped current_twist;
        current_twist.header.frame_id = track_frame;
        current_twist.header.stamp = track_time;
        current_twist.twist = segment.twist.twist;
        predicted_poses.start_velocity = current_twist;

        res.predicted_agents_poses.push_back(predicted_poses);
      }
    }
  }

  return true;
}

bool AgentPathPrediction::predictAgentsExternal(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) {
  // Using external paths
  if (external_paths_) {
    auto external_paths = external_paths_;
    auto tracked_agents = tracked_agents_;

    std::vector<AgentPathVel> agent_path_vel_array;
    for (const auto &path : external_paths->paths) {
      AgentPathVel agent_path_vel{.id = path.id, .path = path.path};

      // set starting velocity of the agent if we find them
      // we do not add current pose at first pose in this case
      for (auto &agent : tracked_agents.agents) {
        if (agent.track_id == path.id) {
          for (auto &segment : agent.segments) {
            if (segment.type == default_agent_part_) {
              agent_path_vel.start_vel = segment.twist;
              break;
            }
          }
          break;
        }
      }
      agent_path_vel_array.push_back(agent_path_vel);
    }
    return predictAgentsFromPaths(req, res, agent_path_vel_array);
  }

  // Using an external goal
  if (got_external_goal_) {
    auto now = ros::Time::now();
    auto tracked_agents = tracked_agents_;
    std::map<uint64_t, geometry_msgs::PoseStamped> ext_goal;

    // get robot pose
    tf::StampedTransform robot_to_map_tf;
    tf::StampedTransform agent_to_map_tf;
    bool transforms_found = false;
    try {
      tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), robot_to_map_tf);

      std::string agents_frame = "map";
      if (!tracked_agents.header.frame_id.empty()) {
        agents_frame = tracked_agents.header.frame_id;
      }
      tf_.lookupTransform(map_frame_id_, agents_frame, ros::Time(0), agent_to_map_tf);

      transforms_found = true;
    } catch (tf::LookupException &ex) {
      ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
    } catch (tf::ConnectivityException &ex) {
      ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
    } catch (tf::ExtrapolationException &ex) {
      ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
    }

    // first check if path calculation is needed, and for whom
    std::vector<AgentStartPoseVel> agent_start_pose_vels;
    std::vector<bool> start_poses_far;
    int idx_order = 0;
    for (auto &agent : tracked_agents.agents) {
      path_vels_pos_.push_back(-1);
      if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
        continue;
      }
      bool path_exist = false;
      for (auto &ex_gl : external_goals_) {
        if (ex_gl.id == agent.track_id) {
          ext_goal[ex_gl.id] = ex_gl.pose;
          break;
        }
      }
      for (const auto &path_vel : path_vels_) {
        if (path_vel.id == agent.track_id) {
          path_exist = true;
          break;
        }
      }

      // get agent pose
      for (auto &segment : agent.segments) {
        if (segment.type == default_agent_part_) {
          geometry_msgs::PoseStamped agent_start;
          agent_start.header.frame_id = tracked_agents.header.frame_id;
          agent_start.header.stamp = now;
          agent_start.pose = segment.pose.pose;

          tf::Pose start_pose_tf;
          start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
          geometry_msgs::Pose start_pose;
          start_pose.orientation.w = 1.0;
          tf::poseMsgToTF(agent_start.pose, start_pose_tf);
          start_pose_tf = agent_to_map_tf * start_pose_tf;
          tf::poseTFToMsg(start_pose_tf, start_pose);

          if (!path_exist) {
            AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
            agent_start_pose_vels.push_back(agent_start_pose_vel);
            path_vels_pos_[agent.track_id - 1] = idx_order;
          } else {
            if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) != req.ids.end()) {
              double dist_far = std::hypot(agent_start.pose.position.x - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.x,
                                           agent_start.pose.position.y - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.y);
              if (dist_far > RECALC_DIST) {  // To ensure that the path is recalculated only if the agent is deviating from the path
                start_poses_far.push_back(true);
                AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
                agent_start_pose_vels.push_back(agent_start_pose_vel);
                path_vels_pos_[agent.track_id - 1] = idx_order;
                path_vels_.clear();
              }
            }
          }
          break;
        }
      }
      idx_order++;
    }

    if (!agent_start_pose_vels.empty()) {
      if (transforms_found) {
        for (auto &agent_start_pose_vel : agent_start_pose_vels) {
          nav_msgs::GetPlan get_plan_srv;
          if (ext_goal.find(agent_start_pose_vel.id) == ext_goal.end()) continue;
          // get agent pose in map frame
          tf::Pose start_pose_tf;
          start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
          tf::poseMsgToTF(agent_start_pose_vel.pose.pose, start_pose_tf);
          start_pose_tf = agent_to_map_tf * start_pose_tf;
          auto start_pose_stamped = agent_start_pose_vel.pose;
          tf::poseTFToMsg(start_pose_tf, start_pose_stamped.pose);
          auto start_path = setFixedPath(start_pose_stamped);

          get_plan_srv.request.start.header.frame_id = map_frame_id_;
          get_plan_srv.request.start.header.stamp = now;
          get_plan_srv.request.start.pose = start_path.poses.back().pose;
          front_pose_pub_.publish(start_path.poses.back());

          get_plan_srv.request.goal.header.frame_id = map_frame_id_;
          get_plan_srv.request.goal.header.stamp = now;
          get_plan_srv.request.goal.pose.position.x = ext_goal[agent_start_pose_vel.id].pose.position.x;
          get_plan_srv.request.goal.pose.position.y = ext_goal[agent_start_pose_vel.id].pose.position.y;
          get_plan_srv.request.goal.pose.position.z = ext_goal[agent_start_pose_vel.id].pose.position.z;
          get_plan_srv.request.goal.pose.orientation = ext_goal[agent_start_pose_vel.id].pose.orientation;

          ROS_DEBUG_NAMED(NODE_NAME,
                          "agent start: x=%.2f, y=%.2f, theta=%.2f, "
                          "goal: x=%.2f, y=%.2f, theta=%.2f",
                          get_plan_srv.request.start.pose.position.x, get_plan_srv.request.start.pose.position.y, tf::getYaw(get_plan_srv.request.start.pose.orientation),
                          get_plan_srv.request.goal.pose.position.x, get_plan_srv.request.goal.pose.position.y, tf::getYaw(get_plan_srv.request.goal.pose.orientation));

          // make plan for agent
          if (get_plan_client_) {
            if (get_plan_client_.call(get_plan_srv)) {
              if (!get_plan_srv.response.plan.poses.empty()) {
                AgentPathVel agent_path_vel;
                agent_path_vel.id = agent_start_pose_vel.id;
                agent_path_vel.path = get_plan_srv.response.plan;
                agent_path_vel.start_vel = agent_start_pose_vel.vel;
                path_vels_.push_back(agent_path_vel);
                got_new_agent_paths_ = true;
              } else {
                ROS_WARN_NAMED(NODE_NAME,
                               "Got empty path for agent, start or "
                               "goal position is probably invalid");
              }
            } else {
              ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service", get_plan_srv_name_.c_str());
            }
          } else {
            ROS_WARN_NAMED(NODE_NAME, "%s service does not exist, re-trying to subscribe", get_plan_srv_name_.c_str());
            ros::NodeHandle private_nh("~/");
            get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name_, true);
          }
        }
      }
    }
    return predictAgentsFromPaths(req, res, path_vels_);
  }

  std::vector<AgentPathVel> empty_path_vels;
  return predictAgentsFromPaths(req, res, empty_path_vels);
}

bool AgentPathPrediction::predictAgentsBehind(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) {
  auto now = ros::Time::now();
  auto tracked_agents = tracked_agents_;

  // get robot pose
  tf::StampedTransform robot_to_map_tf;
  tf::StampedTransform agent_to_map_tf;
  bool transforms_found = false;
  try {
    tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), robot_to_map_tf);
    std::string agents_frame = "map";
    if (!tracked_agents.header.frame_id.empty()) {
      agents_frame = tracked_agents.header.frame_id;
    }
    tf_.lookupTransform(map_frame_id_, agents_frame, ros::Time(0), agent_to_map_tf);

    transforms_found = true;
  } catch (tf::LookupException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<AgentStartPoseVel> agent_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;
  for (auto &agent : tracked_agents.agents) {
    path_vels_pos_.push_back(-1);
    if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
      continue;
    }
    bool path_exist = false;
    for (const auto &path_vel : path_vels_) {
      if (path_vel.id == agent.track_id) {
        path_exist = true;
        break;
      }
    }

    // get agent pose
    for (auto &segment : agent.segments) {
      if (segment.type == default_agent_part_) {
        geometry_msgs::PoseStamped agent_start;
        agent_start.header.frame_id = tracked_agents.header.frame_id;
        agent_start.header.stamp = now;
        agent_start.pose = segment.pose.pose;

        tf::Pose start_pose_tf;
        start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        geometry_msgs::Pose start_pose;
        start_pose.orientation.w = 1.0;
        tf::poseMsgToTF(agent_start.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        tf::poseTFToMsg(start_pose_tf, start_pose);

        if (!path_exist) {
          AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
          agent_start_pose_vels.push_back(agent_start_pose_vel);
          path_vels_pos_[agent.track_id - 1] = idx_order;
        } else {
          if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) != req.ids.end()) {
            double dist_far = std::hypot(agent_start.pose.position.x - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.x,
                                         agent_start.pose.position.y - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.y);

            if (dist_far > RECALC_DIST) {  // To ensure that the path is recalculated only if the agent is deviating from the path
              start_poses_far.push_back(true);
              AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
              agent_start_pose_vels.push_back(agent_start_pose_vel);
              path_vels_pos_[agent.track_id - 1] = idx_order;
              path_vels_.clear();
            }
          }
        }
        break;
      }
    }
    idx_order++;
  }
  if (!agent_start_pose_vels.empty()) {
    if (transforms_found) {
      for (auto &agent_start_pose_vel : agent_start_pose_vels) {
        nav_msgs::GetPlan get_plan_srv;

        auto hum_id = agent_start_pose_vel.id;
        // get agent pose in map frame
        tf::Pose start_pose_tf;
        start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        tf::poseMsgToTF(agent_start_pose_vel.pose.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        auto start_pose_stamped = agent_start_pose_vel.pose;
        tf::poseTFToMsg(start_pose_tf, start_pose_stamped.pose);
        auto start_path = setFixedPath(start_pose_stamped);

        get_plan_srv.request.start.header.frame_id = map_frame_id_;
        get_plan_srv.request.start.header.stamp = now;
        get_plan_srv.request.start.pose = start_path.poses.back().pose;
        front_pose_pub_.publish(start_path.poses.back());

        // calculate agent pose behind robot
        if (!check_path_) {
          check_path_ = true;
          tf::Transform behind_tr;
          behind_tr.setOrigin(tf::Vector3(-agent_dist_behind_robot_, 0.0, 0.0));
          behind_tr.setRotation(tf::createQuaternionFromYaw(agent_angle_behind_robot_));
          behind_tr = robot_to_map_tf * behind_tr;
          tf::transformTFToMsg(behind_tr, behind_pose_);
        }
        get_plan_srv.request.goal.header.frame_id = map_frame_id_;
        get_plan_srv.request.goal.header.stamp = now;
        get_plan_srv.request.goal.pose.position.x = behind_pose_.translation.x;
        get_plan_srv.request.goal.pose.position.y = behind_pose_.translation.y;
        get_plan_srv.request.goal.pose.position.z = behind_pose_.translation.z;
        get_plan_srv.request.goal.pose.orientation = behind_pose_.rotation;

        ROS_DEBUG_NAMED(NODE_NAME,
                        "agent start: x=%.2f, y=%.2f, theta=%.2f, "
                        "goal: x=%.2f, y=%.2f, theta=%.2f",
                        get_plan_srv.request.start.pose.position.x, get_plan_srv.request.start.pose.position.y, tf::getYaw(get_plan_srv.request.start.pose.orientation),
                        get_plan_srv.request.goal.pose.position.x, get_plan_srv.request.goal.pose.position.y, tf::getYaw(get_plan_srv.request.goal.pose.orientation));

        // make plan for agent
        if (get_plan_client_) {
          if (get_plan_client_.call(get_plan_srv)) {
            if (!get_plan_srv.response.plan.poses.empty()) {
              AgentPathVel agent_path_vel;
              agent_path_vel.id = agent_start_pose_vel.id;
              agent_path_vel.path = get_plan_srv.response.plan;
              agent_path_vel.start_vel = agent_start_pose_vel.vel;
              path_vels_.push_back(agent_path_vel);
              got_new_agent_paths_ = true;
            } else {
              ROS_WARN_NAMED(NODE_NAME,
                             "Got empty path for agent, start or "
                             "goal position is probably invalid");
            }
          } else {
            ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service", get_plan_srv_name_.c_str());
          }
        } else {
          ROS_WARN_NAMED(NODE_NAME, "%s service does not exist, re-trying to subscribe", get_plan_srv_name_.c_str());
          ros::NodeHandle private_nh("~/");
          get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name_, true);
        }
      }
    }
  }

  return predictAgentsFromPaths(req, res, path_vels_);
}

bool AgentPathPrediction::predictAgentsGoal(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) {
  auto now = ros::Time::now();
  auto tracked_agents = tracked_agents_;
  std::map<int, geometry_msgs::Pose> predicted_goals;

  for (auto &goal : predicted_goals_.goals) {
    predicted_goals[goal.id] = goal.goal;
  }

  // get robot pose
  tf::StampedTransform robot_to_map_tf;
  tf::StampedTransform agent_to_map_tf;
  bool transforms_found = false;
  try {
    tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), robot_to_map_tf);
    std::string agents_frame = "map";
    if (!tracked_agents.header.frame_id.empty()) {
      agents_frame = tracked_agents.header.frame_id;
    }
    tf_.lookupTransform(map_frame_id_, agents_frame, ros::Time(0), agent_to_map_tf);

    transforms_found = true;
  } catch (tf::LookupException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<AgentStartPoseVel> agent_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;

  for (auto &agent : tracked_agents.agents) {
    path_vels_pos_.push_back(-1);
    if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) == req.ids.end()) {
      continue;
    }
    bool path_exist = false;
    for (const auto &path_vel : path_vels_) {
      if (path_vel.id == agent.track_id) {
        path_exist = true;
        break;
      }
    }

    // get agent pose
    for (auto &segment : agent.segments) {
      if (segment.type == default_agent_part_) {
        geometry_msgs::PoseStamped agent_start;
        agent_start.header.frame_id = tracked_agents.header.frame_id;
        agent_start.header.stamp = now;
        agent_start.pose = segment.pose.pose;

        tf::Pose start_pose_tf;
        start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        geometry_msgs::Pose start_pose;
        start_pose.orientation.w = 1.0;
        tf::poseMsgToTF(agent_start.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        tf::poseTFToMsg(start_pose_tf, start_pose);

        if (!path_exist || predicted_goals_.header.stamp.toSec() < 1) {
          AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
          agent_start_pose_vels.push_back(agent_start_pose_vel);
          path_vels_pos_[agent.track_id - 1] = idx_order;
        } else {
          if (std::find(req.ids.begin(), req.ids.end(), agent.track_id) != req.ids.end()) {
            double dist_far = std::hypot(agent_start.pose.position.x - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.x,
                                         agent_start.pose.position.y - path_vels_[path_vels_pos_[agent.track_id - 1]].path.poses[0].pose.position.y);

            if (dist_far > RECALC_DIST) {
              start_poses_far.push_back(true);
              AgentStartPoseVel agent_start_pose_vel = {.id = agent.track_id, .pose = agent_start, .vel = segment.twist};
              agent_start_pose_vels.push_back(agent_start_pose_vel);
              path_vels_pos_[agent.track_id - 1] = idx_order;
              path_vels_.clear();
            }
          }
        }
        break;
      }
    }
    idx_order++;
  }

  if (!agent_start_pose_vels.empty()) {
    if (transforms_found) {
      for (auto &agent_start_pose_vel : agent_start_pose_vels) {
        nav_msgs::GetPlan get_plan_srv;

        // get agent pose in map frame
        tf::Pose start_pose_tf;
        start_pose_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        tf::poseMsgToTF(agent_start_pose_vel.pose.pose, start_pose_tf);
        start_pose_tf = agent_to_map_tf * start_pose_tf;
        auto start_pose_stamped = agent_start_pose_vel.pose;
        tf::poseTFToMsg(start_pose_tf, start_pose_stamped.pose);
        auto start_path = setFixedPath(start_pose_stamped);

        get_plan_srv.request.start.header.frame_id = map_frame_id_;
        get_plan_srv.request.start.header.stamp = now;
        get_plan_srv.request.start.pose = start_path.poses.back().pose;
        front_pose_pub_.publish(start_path.poses.back());

        get_plan_srv.request.goal.header.frame_id = map_frame_id_;
        get_plan_srv.request.goal.header.stamp = now;
        get_plan_srv.request.goal.pose = predicted_goals[agent_start_pose_vel.id];

        ROS_DEBUG_NAMED(NODE_NAME,
                        "agent start: x=%.2f, y=%.2f, theta=%.2f, "
                        "goal: x=%.2f, y=%.2f, theta=%.2f",
                        get_plan_srv.request.start.pose.position.x, get_plan_srv.request.start.pose.position.y, tf::getYaw(get_plan_srv.request.start.pose.orientation),
                        get_plan_srv.request.goal.pose.position.x, get_plan_srv.request.goal.pose.position.y, tf::getYaw(get_plan_srv.request.goal.pose.orientation));

        // make plan for agent
        if (get_plan_client_) {
          if (get_plan_client_.call(get_plan_srv)) {
            if (!get_plan_srv.response.plan.poses.empty()) {
              AgentPathVel agent_path_vel;
              agent_path_vel.id = agent_start_pose_vel.id;
              agent_path_vel.path = get_plan_srv.response.plan;
              agent_path_vel.start_vel = agent_start_pose_vel.vel;
              path_vels_.push_back(agent_path_vel);
              got_new_agent_paths_ = true;
            } else {
              ROS_WARN_NAMED(NODE_NAME,
                             "Got empty path for agent, start or "
                             "goal position is probably invalid");
            }
          } else {
            ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service", get_plan_srv_name_.c_str());
          }
        } else {
          ROS_WARN_NAMED(NODE_NAME, "%s service does not exist, re-trying to subscribe", get_plan_srv_name_.c_str());
          ros::NodeHandle private_nh("~/");
          get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name_, true);
        }
      }
    }
  }

  return predictAgentsFromPaths(req, res, path_vels_);
}

bool AgentPathPrediction::predictAgentsFromPaths(agent_path_prediction::AgentPosePredict::Request & /*req*/, agent_path_prediction::AgentPosePredict::Response &res,
                                                 const std::vector<AgentPathVel> &path_vels) {
  auto tracked_agents = tracked_agents_;

  if (got_new_agent_paths_) {
    for (auto agent_path_vel : path_vels) {
      auto &poses = agent_path_vel.path.poses;
      if (!poses.empty()) {
        agent_path_prediction::PredictedPoses predicted_poses;
        predicted_poses.id = agent_path_vel.id;

        auto lin_vel = std::hypot(agent_path_vel.start_vel.twist.linear.x, agent_path_vel.start_vel.twist.linear.y);
        auto now = ros::Time::now();

        predicted_poses.poses.resize(poses.size());
        for (size_t i = 0; i < poses.size(); ++i) {
          auto &pose = poses[i];
          geometry_msgs::PoseWithCovarianceStamped predicted_pose;
          if (i == 0 || lin_vel == 0.0) {
            predicted_pose.header.stamp = now;
          } else {
            auto &last_pose = poses[i - 1];
            auto dist = std::hypot(pose.pose.position.x - last_pose.pose.position.x, pose.pose.position.y - last_pose.pose.position.y);
            predicted_pose.header.stamp = predicted_poses.poses[i - 1].header.stamp + ros::Duration(dist / lin_vel);
          }
          predicted_pose.header.frame_id = pose.header.frame_id;
          predicted_pose.pose.pose = pose.pose;
          predicted_poses.poses[i] = predicted_pose;
        }

        for (auto it = last_predicted_poses_.begin(); it != last_predicted_poses_.end(); ++it) {
          if (it->id == predicted_poses.id) {
            last_predicted_poses_.erase(it);
            break;
          }
        }
        last_predicted_poses_.push_back(predicted_poses);

        last_prune_indices_.erase(predicted_poses.id);

        // for (auto it = tracked_agents.agents.begin(); it != tracked_agents.agents.end(); ++it) {  // TODD: Check this, remove for now
        //   if (it->track_id == predicted_poses.id) {
        //     tracked_agents.agents.erase(it);
        //     break;
        //   }
        // }
        ROS_DEBUG_NAMED(NODE_NAME, "Processed new path for agent %ld with %ld poses in frame %s", agent_path_vel.id, predicted_poses.poses.size(),
                        predicted_poses.poses.front().header.frame_id.c_str());
      }
    }
  }
  got_new_agent_paths_ = false;

  for (auto &poses : last_predicted_poses_) {
    if (!poses.poses.empty()) {
      geometry_msgs::PoseStamped start_pose;
      geometry_msgs::TwistStamped start_twist;
      if (transformPoseTwist(tracked_agents, poses.id, poses.poses.front().header.frame_id, start_pose, start_twist)) {
        auto last_prune_index_it = last_prune_indices_.find(poses.id);
        auto begin_index = (last_prune_index_it != last_prune_indices_.end()) ? last_prune_index_it->second : 0;
        auto prune_index = prunePath(begin_index, start_pose.pose, poses.poses);
        last_prune_indices_[poses.id] = prune_index;
        if (prune_index < 0 || prune_index > poses.poses.size()) {
          ROS_ERROR_NAMED(NODE_NAME, "Logical error, cannot prune path");
          continue;
        }
        geometry_msgs::PoseWithCovarianceStamped start_pose_co;
        start_pose_co.header.stamp = start_pose.header.stamp;
        start_pose_co.header.frame_id = start_pose.header.frame_id;
        start_pose_co.pose.pose = start_pose.pose;
        std::vector<geometry_msgs::PoseWithCovarianceStamped> pruned_path;
        pruned_path.push_back(start_pose_co);
        pruned_path.insert(pruned_path.end(), poses.poses.begin() + prune_index, poses.poses.end());

        if (!pruned_path.empty()) {
          // update time stamps for the predicted path
          auto lin_vel = std::hypot(start_twist.twist.linear.x, start_twist.twist.linear.y);
          auto now = ros::Time::now();
          for (size_t i = 0; i < pruned_path.size(); i++) {
            if (i == 0 || lin_vel == 0) {
              pruned_path[i].header.stamp = now;
            } else {
              auto &pose = pruned_path[i].pose.pose;
              auto &last_pose = pruned_path[i - 1].pose.pose;
              auto dist = std::hypot(pose.position.x - last_pose.position.x, pose.position.y - last_pose.position.y);
              pruned_path[i].header.stamp = pruned_path[i - 1].header.stamp + ros::Duration(dist / lin_vel);
            }
          }

          agent_path_prediction::PredictedPoses predicted_poses;
          predicted_poses.id = poses.id;
          predicted_poses.start_velocity = start_twist;
          predicted_poses.poses = pruned_path;

          res.predicted_agents_poses.push_back(predicted_poses);
          // ROS_INFO("Pushed the poses");
          ROS_DEBUG_NAMED(NODE_NAME, "Giving path of %ld points from %ld points for agent %d", predicted_poses.poses.size(), poses.poses.size(), poses.id);
        }
      }
    }
  }

  return true;
}

// TODO: Remove this and make it a subscriber
bool AgentPathPrediction::setGoal(agent_path_prediction::AgentGoal::Request &req, agent_path_prediction::AgentGoal::Response &res) {
  ROS_DEBUG_NAMED(NODE_NAME, "Received new agent goal");
  got_external_goal_ = true;
  external_goals_.clear();
  path_vels_.clear();
  for (auto &goal : req.goals) {
    external_goals_.push_back(goal);
  }

  res.success = true;
  res.message = "Goal has been set.";
  return true;
}

bool AgentPathPrediction::resetPredictionSrvs(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/) {
  got_new_agent_paths_ = false;
  got_external_goal_ = false;
  last_predicted_poses_.clear();
  path_vels_.clear();
  check_path_ = false;
  behind_pose_ = geometry_msgs::Transform();
  return true;
}

void AgentPathPrediction::setParams(double velobs_mul, double velobs_min_rad, double velobs_max_rad, double velobs_max_rad_time, bool velobs_use_ang) {
  velobs_mul_ = velobs_mul;
  velobs_min_rad_ = velobs_min_rad;
  velobs_max_rad_ = velobs_max_rad;
  velobs_max_rad_time_ = velobs_max_rad_time;
  velobs_use_ang_ = velobs_use_ang;

  ROS_DEBUG_NAMED(NODE_NAME, "parameters set: velobs-mul=%f, velocity-obstacle: min-radius:%f, max-radius:%f, max-radius-time=%f use-ang=%d", velobs_mul_, velobs_min_rad_, velobs_max_rad_,
                  velobs_max_rad_time_, velobs_use_ang_);
}

void AgentPathPrediction::reconfigureCB(agent_path_prediction::AgentPathPredictionConfig &config, uint32_t /*level*/) {
  setParams(config.velobs_mul, config.velobs_min_rad, config.velobs_max_rad, config.velobs_max_rad_time, config.velobs_use_ang);
}

nav_msgs::Path AgentPathPrediction::setFixedPath(const geometry_msgs::PoseStamped &start_pose) {
  nav_msgs::Path path;
  path.header.frame_id = start_pose.header.frame_id;
  path.header.stamp = start_pose.header.stamp;
  path.poses.push_back(start_pose);

  // Extract yaw from quaternion
  double roll;
  double pitch;
  double yaw;
  tf2::Quaternion q_start;
  tf2::fromMsg(start_pose.pose.orientation, q_start);
  tf2::Matrix3x3(q_start).getRPY(roll, pitch, yaw);
  double step_distance = 0.1;   // meters
  double total_distance = 0.5;  // meters

  for (double dist = step_distance; dist <= total_distance; dist += step_distance) {
    geometry_msgs::PoseStamped new_pose = start_pose;
    new_pose.pose.position.x += dist * cos(yaw);
    new_pose.pose.position.y += dist * sin(yaw);
    path.poses.push_back(new_pose);
  }
  return path;
}

void AgentPathPrediction::loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh) {
  private_nh.param("ns", ns_, std::string(""));
  private_nh.param("publish_markers", publish_markers_, true);
  private_nh.param("robot_frame_id", robot_frame_id_, std::string(ROBOT_FRAME_ID));
  private_nh.param("map_frame_id", map_frame_id_, std::string(MAP_FRAME_ID));
  private_nh.param("agent_dist_behind_robot", agent_dist_behind_robot_, AGENT_DIST_BEHIND_ROBOT);
  private_nh.param("agent_angle_behind_robot", agent_angle_behind_robot_, AGENT_ANGLE_BEHIND_ROBOT);
  private_nh.param("tracked_agents_sub_topic", tracked_agents_sub_topic_, std::string(AGENTS_SUB_TOPIC));
  private_nh.param("external_paths_sub_topic", external_paths_sub_topic_, std::string(EXTERNAL_PATHS_SUB_TOPIC));
  private_nh.param("predicted_goal_topic", predicted_goal_topic_, std::string(PREDICTED_GOAL_SUB_TOPIC));
  private_nh.param("get_plan_srv_name", get_plan_srv_name_, std::string(GET_PLAN_SRV_NAME));
  private_nh.param("default_agent_part", default_agent_part_, static_cast<int>(DEFAULT_AGENT_PART));
}

size_t AgentPathPrediction::prunePath(size_t begin_index, const geometry_msgs::Pose &pose, const std::vector<geometry_msgs::PoseWithCovarianceStamped> &path) {
  size_t prune_index = begin_index;
  double x_diff;
  double y_diff;
  double sq_diff;
  double smallest_sq_diff = std::numeric_limits<double>::max();
  while (begin_index < path.size()) {
    x_diff = path[begin_index].pose.pose.position.x - pose.position.x;
    y_diff = path[begin_index].pose.pose.position.y - pose.position.y;
    sq_diff = x_diff * x_diff + y_diff * y_diff;
    if (sq_diff < smallest_sq_diff) {
      prune_index = begin_index;
      smallest_sq_diff = sq_diff;
    }
    ++begin_index;
  }
  return prune_index;
}

bool AgentPathPrediction::transformPoseTwist(const cohan_msgs::TrackedAgents &tracked_agents, const uint64_t &agent_id, const std::string &to_frame, geometry_msgs::PoseStamped &pose,
                                             geometry_msgs::TwistStamped &twist) const {
  for (const auto &agent : tracked_agents.agents) {
    if (agent.track_id == agent_id) {
      for (const auto &segment : agent.segments) {
        if (segment.type == default_agent_part_) {
          geometry_msgs::PoseStamped pose_ut;
          pose_ut.header.stamp = tracked_agents.header.stamp;
          pose_ut.header.frame_id = tracked_agents.header.frame_id;
          pose_ut.pose = segment.pose.pose;
          twist.header.stamp = tracked_agents.header.stamp;
          twist.header.frame_id = tracked_agents.header.frame_id;
          twist.twist = segment.twist.twist;
          try {
            tf::Stamped<tf::Pose> pose_tf;
            tf::poseStampedMsgToTF(pose_ut, pose_tf);
            tf::StampedTransform start_pose_to_plan_transform;
            if (to_frame.empty() || pose_ut.header.frame_id.empty() || twist.header.frame_id.empty()) {
              continue;
            }
            tf_.waitForTransform(to_frame, pose_ut.header.frame_id, ros::Time(0), ros::Duration(0.5));
            tf_.lookupTransform(to_frame, pose_ut.header.frame_id, ros::Time(0), start_pose_to_plan_transform);
            pose_tf.setData(start_pose_to_plan_transform * pose_tf);
            pose_tf.frame_id_ = to_frame;
            tf::poseStampedTFToMsg(pose_tf, pose);

            geometry_msgs::Twist start_twist_to_plan_transform;
            tf_.lookupTwist(to_frame, twist.header.frame_id, ros::Time::now(), ros::Duration(0.1), start_twist_to_plan_transform);
            twist.twist.linear.x -= start_twist_to_plan_transform.linear.x;
            twist.twist.linear.y -= start_twist_to_plan_transform.linear.y;
            twist.twist.angular.z -= start_twist_to_plan_transform.angular.z;
            twist.header.frame_id = to_frame;
            return true;
          } catch (tf::LookupException &ex) {
            ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
          } catch (tf::ConnectivityException &ex) {
            ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
          } catch (tf::ExtrapolationException &ex) {
            ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
          }
          break;
        }
      }
      break;
    }
  }
  return false;
}

}  // namespace agents

// handler for something to do before killing the node
void sigintHandler(int sig) {
  ROS_DEBUG_NAMED(NODE_NAME, "node %s will now shutdown", NODE_NAME);

  // the default sigint handler, it calls shutdown() on node
  ros::shutdown();
}
#if !defined(DOXYGEN_SHOULD_SKIP_THIS)
// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv) {
  // starting the optotrack_person node
  ros::init(argc, argv, NODE_NAME);
  ROS_DEBUG_NAMED(NODE_NAME, "started %s node", NODE_NAME);

  // initiazling agent_path_prediction class
  agents::AgentPathPrediction agent_path_prediction;
  agent_path_prediction.initialize();

  agents::PredictGoalROS predict_srv;

  // look for sigint and start spinning the node
  signal(SIGINT, sigintHandler);
  ros::spin();

  return 0;
}
#endif
