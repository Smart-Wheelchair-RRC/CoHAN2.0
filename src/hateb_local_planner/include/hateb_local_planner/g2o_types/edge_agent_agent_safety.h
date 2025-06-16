/*
 * Copyright (c) 2024 LAAS/CNRS
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
 * Authors: Phani Teja Singamaneni
 */

#ifndef EDGE_AGENT_AGENT_SAFETY_H_
#define EDGE_AGENT_AGENT_SAFETY_H_

#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/hateb_config.h>

namespace hateb_local_planner {

/**
 * @class EdgeAgentAgentSafety
 * @brief Edge defining a safety constraint between two agent poses (e.g., humans) in the optimization graph.
 *
 * This edge penalizes configurations where two agents come closer than a specified minimum distance.
 * The cost is computed as a quadratic penalty if the distance between the agents (minus their radii) is less than
 * cfg_->hateb.min_agent_agent_dist. The edge is used to enforce social distancing between agents.
 *
 * Inherits from BaseTebBinaryEdge<1, double, VertexPose, VertexPose>.
 */
class EdgeAgentAgentSafety : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
 public:
  /**
   * @brief Construct edge and set measurement to zero (unused).
   */
  EdgeAgentAgentSafety() { this->setMeasurement(0.); }

  /**
   * @brief Compute the error / cost for the edge.
   *
   * Calculates the external distance between two agent poses, subtracts twice the agent radius,
   * and applies a quadratic penalty if the result is below the minimum allowed agent-agent distance.
   *
   * The error is stored in _error[0].
   */
  void computeError() override {
    ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig() on EdgeAgentAgentSafety()");
    const auto *agent1_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const auto *agent2_bandpt = static_cast<const VertexPose *>(_vertices[1]);

    double agent_radius = cfg_->human_model->getCircumscribedRadius();
    // Get the distance
    double dist = std::hypot(agent1_bandpt->x() - agent2_bandpt->x(), agent1_bandpt->y() - agent2_bandpt->y()) - (2 * agent_radius);

    ROS_DEBUG_THROTTLE(0.5, "agent agent external dist = %f", dist);
    _error[0] = penaltyBoundFromBelowQuad(dist, cfg_->hateb.min_agent_agent_dist, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAgentAgentSafety::computeError() _error[0]=%f\n", _error[0]);
  }

 public:
  /**
   * @brief Ensure proper alignment for fixed-size Eigen types.
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace hateb_local_planner

#endif  // EDGE_AGENT_AGENT_SAFETY_H_
