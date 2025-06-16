/*
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Phani Teja Singamaneni
 */

#ifndef EDGE_AGENT_ROBOT_SAFETY_H_
#define EDGE_AGENT_ROBOT_SAFETY_H_

#include <hateb_local_planner/footprint_model.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/obstacles.h>

namespace hateb_local_planner {

/**
 * @class EdgeAgentRobotSafety
 * @brief Edge defining a safety constraint between a robot and an agent (e.g., human) in the optimization graph.
 *
 * This edge penalizes configurations where the robot and an agent come closer than a specified minimum distance.
 * The cost is computed as a quadratic penalty if the distance between the robot (using its footprint model) and the agent (using its radius) is less than
 * cfg_->hateb.min_agent_robot_dist. The edge is used to enforce social distancing or collision avoidance between the robot and agents.
 *
 * Inherits from BaseTebBinaryEdge<1, double, VertexPose, VertexPose>.
 */
class EdgeAgentRobotSafety : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge and set measurement to zero (unused).
   */
  EdgeAgentRobotSafety() { this->setMeasurement(0.); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the distance between the robot and agent, subtracts the agent radius, and applies a quadratic penalty if the result is below the minimum allowed agent-robot distance.
   * The error is stored in _error[0].
   */
  void computeError() override {
    ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig() on EdgeAgentRobotSafety()");
    const auto *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const auto *agent_bandpt = static_cast<const VertexPose *>(_vertices[1]);
    static_cast<PointObstacle *>(obs_)->setCentroid(agent_bandpt->x(), agent_bandpt->y());

    // The distance is taken according to the footprint model of the robot. Could have some errors
    double agent_radius = cfg_->human_model->getCircumscribedRadius();
    double dist = cfg_->robot_model->calculateDistance(robot_bandpt->pose(), obs_) - agent_radius;

    // Apply a quadratic penalty
    _error[0] = penaltyBoundFromBelowQuad(dist, cfg_->hateb.min_agent_robot_dist, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAgentRobotSafety::computeError() _error[0]=%f\n", _error[0]);
  }

 protected:
  /**
   * @brief Obstacle pointer used to represent the agent as a point obstacle for distance calculation.
   */
  Obstacle *obs_ = new PointObstacle();

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace hateb_local_planner

#endif
