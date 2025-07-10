/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2020 LAAS-CNRS
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

#ifndef AGENT_PATHARRAY_DISPLAY_H
#define AGENT_PATHARRAY_DISPLAY_H

// #include <nav_msgs/Path.h>
#include <cohan_msgs/AgentPath.h>
#include <cohan_msgs/AgentPathArray.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>

#include "rviz/message_filter_display.h"

namespace Ogre {
class ManualObject;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;
}  // namespace rviz

namespace rviz_path_array {

class AgentPathArrayDisplay;
/**
 * \class AgentPathArrayDisplay
 * \brief Displays a cohan_msgs::AgentPathArray message
 */
class AgentPathArrayDisplay : public rviz::MessageFilterDisplay<cohan_msgs::AgentPathArray> {
  Q_OBJECT
 public:
  AgentPathArrayDisplay();
  ~AgentPathArrayDisplay() override;

  /** @brief Overridden from Display. */
  void reset() override;

 protected:
  /** @brief Overridden from Display. */
  void onInitialize() override;

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage(const cohan_msgs::AgentPathArray::ConstPtr& msg) override;

 private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowColor();
  void updatePoseArrowGeometry();

 private:
  void destroyObjects();
  void allocateArrowVector(std::vector<rviz::Arrow*>& arrow_vect, int num);
  void allocateAxesVector(std::vector<rviz::Axes*>& axes_vect, int num);
  void destroyPoseAxesChain();
  void destroyPoseArrowChain();

  std::vector<Ogre::ManualObject*> manual_objects_;
  std::vector<rviz::BillboardLine*> billboard_lines_;
  std::vector<std::vector<rviz::Axes*> > axes_chain_;
  std::vector<std::vector<rviz::Arrow*> > arrow_chain_;

  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* line_width_property_;
  rviz::IntProperty* buffer_length_property_;
  rviz::VectorProperty* offset_property_;

  enum LineStyle { LINES, BILLBOARDS };

  // pose marker property
  rviz::EnumProperty* pose_style_property_;
  rviz::FloatProperty* pose_axes_length_property_;
  rviz::FloatProperty* pose_axes_radius_property_;
  rviz::ColorProperty* pose_arrow_color_property_;
  rviz::FloatProperty* pose_arrow_shaft_length_property_;
  rviz::FloatProperty* pose_arrow_head_length_property_;
  rviz::FloatProperty* pose_arrow_shaft_diameter_property_;
  rviz::FloatProperty* pose_arrow_head_diameter_property_;

  enum PoseStyle {
    NONE,
    AXES,
    ARROWS,
  };
};

}  // namespace rviz_path_array

#endif /* AGENT_PATHARRAY_DISPLAY_H */
