/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 * Copyright (c) 2020 LAAS/CNRS
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
 * Author: Christoph Rösmann
 * Modified by: Phani Teja Singamaneni
 *********************************************************************/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

// teb stuff
#include <hateb_local_planner/footprint_model.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/timed_elastic_band.h>

// ros stuff
#include <base_local_planner/goal_functions.h>
#include <ros/publisher.h>
#include <tf/transform_listener.h>

// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

// messages
#include <cohan_msgs/AgentPathArray.h>
#include <cohan_msgs/AgentTimeToGoal.h>
#include <cohan_msgs/AgentTimeToGoalArray.h>
#include <cohan_msgs/AgentTrajectoryArray.h>
#include <cohan_msgs/TrackedAgents.h>
#include <cohan_msgs/TrackedSegmentType.h>
#include <cohan_msgs/TrajectoryPoint.h>
#include <cohan_msgs/TrajectoryStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hateb_local_planner {

typedef struct {
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<cohan_msgs::TrajectoryPoint> optimized_trajectory;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} PlanTrajCombined;

typedef struct {
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<geometry_msgs::PoseStamped> plan_to_optimize;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} PlanCombined;

typedef struct {
  uint64_t id;
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<cohan_msgs::TrajectoryPoint> optimized_trajectory;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} AgentPlanTrajCombined;

typedef struct {
  uint64_t id;
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<geometry_msgs::PoseStamped> plan_to_optimize;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} AgentPlanCombined;

class TebOptimalPlanner;  //!< Forward Declaration

/**
 * @class TebVisualization
 * @brief Visualize stuff from the hateb_local_planner
 */
class TebVisualization {
 public:
  /**
   * @brief Default constructor
   * @remarks do not forget to call initialize()
   */
  TebVisualization();

  /**
   * @brief Constructor that initializes the class and registers topics
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the HATebConfig class for parameters
   */
  TebVisualization(ros::NodeHandle& nh, const HATebConfig& cfg);

  /**
   * @brief Initializes the class and registers topics.
   *
   * Call this function if only the default constructor has been called before.
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the HATebConfig class for parameters
   */
  void initialize(ros::NodeHandle& nh, const HATebConfig& cfg);

  /** @name Publish to topics */
  //@{

  /**
   * @brief Publish a given global plan to the ros topic \e ../../global_plan
   * @param global_plan Pose array describing the global plan
   */
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const;

  /**
   * @brief Publish a given agent's global plan to the ros topic
   * @param agents_plans Vector comntaining the AgentPlanCombined
   */
  void publishAgentGlobalPlans(const std::vector<AgentPlanCombined>& agents_plans) const;

  /**
   * @brief Publish a given local plan to the ros topic \e ../../local_plan
   * @param local_plan Pose array describing the local plan
   */
  void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const;

  /**
   * @brief Publish visualization markers for tracked agents in the environment
   * @param agents Pointer to the tracked agents message containing agent positions, velocities, and properties
   */
  void publishTrackedAgents(const cohan_msgs::TrackedAgentsConstPtr& agents);

  /**
   * @brief Publish Timed_Elastic_Band related stuff (local plan, pose sequence).
   *
   * Given a Timed_Elastic_Band instance, publish the local plan to  \e ../../local_plan
   * and the pose sequence to  \e ../../teb_poses.
   * @param teb const reference to a Timed_Elastic_Band
   */
  void publishLocalPlanAndPoses(const TimedElasticBand& teb, const BaseFootprintModel& robot_model, double fp_size, const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish local plans and pose sequences for multiple agents
   *
   * Similar to publishLocalPlanAndPoses but handles multiple agents. Publishes local plans and
   * pose sequences for each agent in the map to their respective ROS topics.
   * @param agents_tebs_map Map of agent IDs to their Timed Elastic Band trajectories
   * @param agent_model The footprint model used for all agents
   * @param fp_size Size of the footprint for visualization
   * @param color Color to use for visualization markers (RGBA)
   */
  void publishAgentLocalPlansAndPoses(const std::map<uint64_t, TimedElasticBand>& agents_tebs_map, const BaseFootprintModel& agent_model, double fp_size,
                                      const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish a complete trajectory including plan segments before and after optimization
   *
   * Publishes visualization of the complete trajectory which includes:
   * - The plan segment before optimization
   * - The optimized trajectory
   * - The plan segment after optimization
   * @param plan_traj_combined Combined plan and trajectory data structure containing all segments
   */
  void publishTrajectory(const PlanTrajCombined& plan_traj_combined);

  /**
   * @brief Publish trajectories for multiple agents
   *
   * Publishes visualization of complete trajectories for multiple agents, each including:
   * - The plan segment before optimization
   * - The optimized trajectory
   * - The plan segment after optimization
   * @param agents_plans_combined Vector of combined plan and trajectory data for each agent
   */
  void publishAgentTrajectories(const std::vector<AgentPlanTrajCombined>& agents_plans_combined);

  /**
   * @brief Publish the visualization of the robot model
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseFootprintModel
   * @param ns Namespace for the marker objects
   * @param color Color of the footprint
   */
  void publishRobotFootprintModel(const PoseSE2& current_pose, const BaseFootprintModel& robot_model, const std::string& ns = "RobotFootprintModel",
                                  const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish the robot footprints related to infeasible poses
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseFootprintModel
   */
  void publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseFootprintModel& robot_model);

  /**
   * @brief Publish obstacle positions to the ros topic \e ../../teb_markers
   * @param obstacles Obstacle container
   */
  void publishObstacles(const ObstContainer& obstacles) const;

  /**
   * @brief Publish via-points to the ros topic \e ../../teb_markers
   * @param via_points via-point container
   */
  void publishViaPoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns = "ViaPoints") const;

  /**
   * @brief Publish a boost::adjacency_list (boost's graph datatype) via markers.
   * @remarks Make sure that vertices of the graph contain a member \c pos as \c Eigen::Vector2d type
   *	      to query metric position values.
   * @param graph Const reference to the boost::adjacency_list (graph)
   * @param ns_prefix Namespace prefix for the marker objects (the strings "Edges" and "Vertices" will be appended)
   * @tparam GraphType boost::graph object in which vertices has the field/member \c pos.
   */
  template <typename GraphType>
  void publishGraph(const GraphType& graph, const std::string& ns_prefix = "Graph");

  /**
   * @brief Publish multiple 2D paths (each path given as a point sequence) from a container class.
   *
   * Provide a std::vector< std::vector< T > > in which T.x() and T.y() exist
   * and std::vector could be individually substituded by std::list / std::deque /...
   *
   * A common point-type for object T could be Eigen::Vector2d.
   *
   * T could be also a raw pointer std::vector< std::vector< T* > >.
   *
   * @code
   * 	typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > PathType; // could be a list or deque as well ...
   *    std::vector<PathType> path_container(2); // init 2 empty paths; the container could be a list or deque as well ...
   * 	// Fill path_container.at(0) with Eigen::Vector2d elements, we skip that here
   * 	// Fill path_container.at(1) with Eigen::Vector2d elements, we skip that here
   *    publishPathContainer( path_container.begin(), path_container.end() );
   * @endcode
   *
   * @remarks Actually the underlying path does not necessarily need to be a Eigen::Vector2d sequence.
   *          Eigen::Vector2d can be replaced with any datatype that implement public x() and y() methods.\n
   * @param first Bidirectional iterator pointing to the begin of the path
   * @param last Bidirectional iterator pointing to the end of the path
   * @param ns Namespace for the marker objects (the strings "Edges" and "Vertices" will be appended)
   * @tparam BidirIter Bidirectional iterator to a 2D path (sequence of Eigen::Vector2d elements) in a container
   */
  template <typename BidirIter>
  void publishPathContainer(BidirIter first, BidirIter last, const std::string& ns = "PathContainer");

  /**
   * @brief Publish multiple Tebs from a container class (publish as marker message).
   *
   * @param teb_planner Container of boost::shared_ptr< TebOptPlannerPtr >
   * @param ns Namespace for the marker objects
   */
  void publishTebContainer(const std::vector<boost::shared_ptr<TebOptimalPlanner> >& teb_planner, const std::string& ns = "TebContainer");

  /**
   * @brief Publish a feedback message (multiple trajectory version)
   *
   * The feedback message contains the all planned trajectory candidates (e.g. if planning in distinctive topologies is turned on).
   * Each trajectory is composed of the sequence of poses, the velocity profile and temporal information.
   * The feedback message also contains a list of active obstacles.
   * @param teb_planners container with multiple tebs (resp. their planner instances)
   * @param selected_trajectory_idx Idx of the currently selected trajectory in \c teb_planners
   * @param obstacles Container of obstacles
   */
  void publishFeedbackMessage(const std::vector<boost::shared_ptr<TebOptimalPlanner> >& teb_planners, unsigned int selected_trajectory_idx, const ObstContainer& obstacles);

  /**
   * @brief Publish a feedback message (single trajectory overload)
   *
   * The feedback message contains the planned trajectory
   * that is composed of the sequence of poses, the velocity profile and temporal information.
   * The feedback message also contains a list of active obstacles.
   * @param teb_planner the planning instance
   * @param obstacles Container of obstacles
   */
  void publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles);

  //@}

  /**
   * @brief Helper function to generate a color message from single values
   * @param a Alpha value
   * @param r Red value
   * @param g Green value
   * @param b Blue value
   * @return Color message
   */
  static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

  static void setMarkerColour(visualization_msgs::Marker& marker, double itr, double n);
  void publishMode(int Mode);

 protected:
  /**
   * @brief Small helper function that checks if initialize() has been called and prints an error message if not.
   * @return \c true if not initialized, \c false if everything is ok
   */
  bool printErrorWhenNotInitialized() const;

  /**
   * @brief Callback function for the clearing timer that removes outdated visualization markers
   * @param event The timer event containing timing information about the callback
   * @details This protected method is called periodically to remove visualization markers that are
   *          no longer needed, preventing visualization clutter and memory buildup
   */
  void clearingTimerCB(const ros::TimerEvent& event);

  ros::Publisher global_plan_pub_;                                   //!< Publisher for the global plan
  ros::Publisher local_plan_pub_;                                    //!< Publisher for the local plan
  ros::Publisher local_traj_pub_;                                    //!< Publisher for the local traj
  ros::Publisher agents_global_plans_pub_;                           //!< Publisher for the local plan
  ros::Publisher agents_local_plans_pub_;                            //!< Publisher for the local plan
  ros::Publisher agents_local_trajs_pub_;                            //!< Publisher for the agents local plans
  ros::Publisher teb_poses_pub_, teb_fp_poses_pub_;                  //!< Publisher for the trajectory pose sequence
  ros::Publisher agents_tebs_poses_pub_, agents_tebs_fp_poses_pub_;  //!< Publisher for the trajectory pose sequence
  ros::Publisher teb_marker_pub_;                                    //!< Publisher for visualization markers
  ros::Publisher feedback_pub_;                                      //!< Publisher for optimization feedback messages
  ros::Publisher mode_text_pub_;                                     //!< Publisher for current planner mode text visualization
  ros::Publisher robot_traj_time_pub_;                               //!< Publisher for robot's time to goal with optimized trajectory (until end of trajectory)
  ros::Publisher robot_path_time_pub_;                               //!< Publisher for robot's full time to goal (until goal, using path + traj)
  ros::Publisher robot_next_pose_pub_;                               //!< Publisher for robot's predicted next pose
  ros::Publisher agent_next_pose_pub_;                               //!< Publisher for agents' predicted next poses
  ros::Publisher agent_trajs_time_pub_;                              //!< Publisher for agents' time to goal with optimized trajectory (until end of trajectory)
  ros::Publisher agent_paths_time_pub_;                              //!< Publisher for agents' full time to goal (until goal, using path + traj)
  ros::Publisher agent_marker_pub_;                                  //!< Publisher for agent visualization markers
  ros::Publisher agent_arrow_pub_;                                   //!< Publisher for agent direction arrow markers
  ros::Subscriber tracked_agents_sub_;                               //!< Subscriber for tracked agents data input
  std::vector<double> vel_robot_;                                    //!< Store robot velocity history (for bar visualization)
  std::vector<double> vel_agent_;                                    //!< Store agent velocity history (for bar visualization)
  tf::TransformListener tf_;                                         //!< Transform listener for coordinate frame transformations
  ros::Publisher ttg_pub_;                                           //!< Publisher for time-to-goal information
  std::string ns_;                                                   //!< Name space of the robot
  std::string tracked_agents_sub_topic_;                             //!< Tracked agents sub topic

  const HATebConfig* cfg_;     //!< Config class that stores and manages all related parameters
  bool initialized_;           //!< Keeps track about the correct initialization of this class
  ros::Timer clearing_timer_;  //!< Timer for periodically clearing old visualization markers

  bool last_publish_robot_global_plan_,          //!< Last publish state of robot's global plan
      last_publish_robot_local_plan_,            //!< Last publish state of robot's local plan
      last_publish_robot_local_plan_poses_,      //!< Last publish state of robot's local plan poses
      last_publish_robot_local_plan_fp_poses_,   //!< Last publish state of robot's local plan footprint poses
      last_publish_agents_global_plans_,         //!< Last publish state of agents' global plans
      last_publish_agents_local_plans_,          //!< Last publish state of agents' local plans
      last_publish_agents_local_plan_poses_,     //!< Last publish state of agents' local plan poses
      last_publish_agents_local_plan_fp_poses_;  //!< Last publish state of agents' local plan footprint poses

  mutable int last_robot_fp_poses_idx_,  //!< Index of last published robot footprint pose
      last_agent_fp_poses_idx_;          //!< Index of last published agent footprint pose

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the TebVisualization
using TebVisualizationPtr = boost::shared_ptr<TebVisualization>;

//! Abbrev. for shared instances of the TebVisualization (read-only)
using TebVisualizationConstPtr = boost::shared_ptr<const TebVisualization>;

}  // namespace hateb_local_planner

// Include template method implementations / definitions
#include <hateb_local_planner/visualization.hpp>

#endif /* VISUALIZATION_H_ */
