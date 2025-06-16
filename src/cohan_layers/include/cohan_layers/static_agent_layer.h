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
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/

#ifndef STATIC_AGENT_LAYER_H
#define STATIC_AGENT_LAYER_H
#include <cohan_layers/AgentStaticLayerConfig.h>
#include <cohan_layers/agent_layer.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace cohan_layers {
class StaticAgentLayer : public AgentLayer {
 public:
  StaticAgentLayer() { layered_costmap_ = nullptr; }

  /**
   * @brief Initializes the StaticAgentLayer and sets up ROS communication
   */
  void onInitialize() override;

  /**
   * @brief Updates the bounds of the costmap based on static agent positions
   * @param min_x Minimum x-coordinate of the bounds
   * @param min_y Minimum y-coordinate of the bounds
   * @param max_x Maximum x-coordinate of the bounds
   * @param max_y Maximum y-coordinate of the bounds
   */
  void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) override;

  /**
   * @brief Updates the costmap with static agent information
   * @param master_grid Reference to the master costmap grid
   * @param min_i Minimum i-index of the grid
   * @param min_j Minimum j-index of the grid
   * @param max_i Maximum i-index of the grid
   * @param max_j Maximum j-index of the grid
   */
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

 protected:
  /**
   * @brief Loads ROS parameters from the node handle
   * @param private_nh Private node handle containing parameters
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);

  /**
   * @brief Configures the StaticAgentLayer using dynamic reconfigure
   * @param config Configuration parameters for the layer
   * @param level Reconfiguration level
   */
  void configure(AgentStaticLayerConfig& config, uint32_t level);

  /**
   * @brief Dynamic reconfigure server for StaticAgentLayer
   */
  dynamic_reconfigure::Server<AgentStaticLayerConfig>* server_;

  /**
   * @brief Callback type for dynamic reconfigure server
   */
  dynamic_reconfigure::Server<AgentStaticLayerConfig>::CallbackType f_;
};
}  // namespace cohan_layers

#endif  // STATIC_AGENT_LAYER_H
