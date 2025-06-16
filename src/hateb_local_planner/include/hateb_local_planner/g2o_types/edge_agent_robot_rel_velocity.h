/*
 * Copyright (c) 2025 LAAS/CNRS
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

#ifndef EDGE_AGENT_ROBOT_REL_VELOCIY_H_
#define EDGE_AGENT_ROBOT_REL_VELOCIY_H_

#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/vertex_timediff.h>
#include <hateb_local_planner/hateb_config.h>

namespace hateb_local_planner {

/**
 * @class EdgeAgentRobotRelVelocity
 * @brief Edge defining a relative velocity constraint between robot and agent trajectories in the optimization graph.
 *
 * This edge penalizes configurations where the relative velocity between the robot and an agent exceeds a specified threshold.
 * The cost is computed based on the difference in velocities projected along the direction between the robot and agent, and is penalized if it exceeds cfg_->hateb.rel_vel_cost_threshold.
 *
 * Inherits from BaseTebMultiEdge<1, double>.
 */
class EdgeAgentRobotRelVelocity : public BaseTebMultiEdge<1, double> {
 public:
  /**
   * @brief Construct edge and resize to 6 vertices (robot/agent poses and time diffs).
   */
  EdgeAgentRobotRelVelocity() { this->resize(6); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the relative velocity between the robot and agent over a time interval, and applies a penalty if the cost exceeds the allowed threshold.
   * The error is stored in _error[0].
   */
  void computeError() override {
    ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig() on EdgeAgentRobotRelVelocity()");
    const auto *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const auto *robot_bandpt_nxt = static_cast<const VertexPose *>(_vertices[1]);
    const auto *dt_robot = static_cast<const VertexTimeDiff *>(_vertices[2]);
    const auto *agent_bandpt = static_cast<const VertexPose *>(_vertices[3]);
    const auto *agent_bandpt_nxt = static_cast<const VertexPose *>(_vertices[4]);
    const auto *dt_agent = static_cast<const VertexTimeDiff *>(_vertices[5]);

    // Get the velocities of the robot and agent
    Eigen::Vector2d diff_robot = robot_bandpt_nxt->position() - robot_bandpt->position();
    Eigen::Vector2d robot_vel = diff_robot / dt_robot->dt();
    Eigen::Vector2d diff_agent = agent_bandpt_nxt->position() - agent_bandpt->position();
    Eigen::Vector2d agent_vel = diff_agent / dt_agent->dt();

    // Get the direction vectors
    Eigen::Vector2d d_rtoh = agent_bandpt->position() - robot_bandpt->position();
    Eigen::Vector2d d_htor = robot_bandpt->position() - agent_bandpt->position();

    // Cost definition
    double rel_vel_cost = (std::max((robot_vel - agent_vel).dot(d_rtoh), 0.0) + robot_vel.norm() + 1) / d_rtoh.norm();

    _error[0] = penaltyBoundFromAbove(rel_vel_cost, cfg_->hateb.rel_vel_cost_threshold, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAgentRobot::computeError() _error[0]=%f\n", _error[0]);
  }

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace hateb_local_planner

#endif  // EDGE_AGENT_ROBOT_REL_VELOCIY_H_
