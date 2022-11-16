// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#include "angles_display.h"

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <QPainter>

namespace jsk_rviz_plugins
{

  AnglesDisplay::AnglesDisplay()
    : rviz::Display(), update_required_(false), sagittal_first_time_(true), axial_first_time_(true),
     sagittal_data_(0.0), axial_data_(0.0), sagittal_target_(0.0), axial_target_(0.0), target_active_(false)
  {
    update_sagittal_topic_property_ = new rviz::RosTopicProperty(
      "Sagittal Topic", "",
      ros::message_traits::datatype<std_msgs::Float32>(),
      "std_msgs::Float32 topic to subscribe to.",
      this, SLOT( updateTopic() ));
    update_axial_topic_property_ = new rviz::RosTopicProperty(
      "Axial Topic", "",
      ros::message_traits::datatype<std_msgs::Float32>(),
      "std_msgs::Float32 topic to subscribe to.",
      this, SLOT( updateTopic() ));
    update_target_topic_property_ = new rviz::RosTopicProperty(
      "Target Topic", "",
      ros::message_traits::datatype<std_msgs::Bool>(),
      "std_msgs::Boolk topic to subscribe to.",
      this, SLOT( updateTopic() ));
    size_property_ = new rviz::IntProperty("size", 400,
                                           "size of the plotter window",
                                           this, SLOT(updateSize()));
    left_property_ = new rviz::IntProperty("left", 15,
                                           "left of the plotter window",
                                           this, SLOT(updateLeft()));
    top_property_ = new rviz::IntProperty("top", 15,
                                          "top of the plotter window",
                                          this, SLOT(updateTop()));
    fg_color_property_ = new rviz::ColorProperty("foreground color",
                                                 QColor(25, 255, 240),
                                                 "color to draw line",
                                                 this, SLOT(updateFGColor()));
    fg_alpha_property_
      = new rviz::FloatProperty("foreground alpha", 0.7,
                                "alpha belnding value for foreground",
                                this, SLOT(updateFGAlpha()));
    fg_alpha2_property_
      = new rviz::FloatProperty("foreground alpha 2", 0.4,
                                "alpha belnding value for foreground for indicator",
                                this, SLOT(updateFGAlpha2()));
    bg_color_property_ = new rviz::ColorProperty("background color",
                                                 QColor(0, 0, 0),
                                                 "background color",
                                                 this, SLOT(updateBGColor()));
    bg_alpha_property_
      = new rviz::FloatProperty("backround alpha", 0.0,
                                "alpha belnding value for background",
                                this, SLOT(updateBGAlpha()));
    text_size_property_
      = new rviz::IntProperty("text size", 14,
                              "text size",
                              this, SLOT(updateTextSize()));
    show_caption_property_
      = new rviz::BoolProperty("show caption", true,
                                "show caption",
                                this, SLOT(updateShowCaption()));
    max_value_property_
      = new rviz::FloatProperty("max value", 180.0,
                                "max value of pie chart",
                                this, SLOT(updateMaxValue()));
    min_value_property_
      = new rviz::FloatProperty("min value", -180.0,
                                "min value of pie chart",
                                this, SLOT(updateMinValue()));
    auto_color_change_property_
      = new rviz::BoolProperty("auto color change",
                               false,
                               "change the color automatically",
                               this, SLOT(updateAutoColorChange()));
    max_color_property_
      = new rviz::ColorProperty("max color",
                                QColor(255, 0, 0),
                                "only used if auto color change is set to True.",
                                this, SLOT(updateMaxColor()));

    clockwise_rotate_property_
      = new rviz::BoolProperty("clockwise rotate direction",
                               false,
                               "change the rotate direction",
                               this, SLOT(updateClockwiseRotate()));
  }

  AnglesDisplay::~AnglesDisplay()
  {
    if (overlay_->isVisible()) {
      overlay_->hide();
    }
    delete update_sagittal_topic_property_;
    delete update_axial_topic_property_;
    delete fg_color_property_;
    delete bg_color_property_;
    delete fg_alpha_property_;
    delete fg_alpha2_property_;
    delete bg_alpha_property_;
    delete top_property_;
    delete left_property_;
    delete size_property_;
    delete min_value_property_;
    delete max_value_property_;
    delete text_size_property_;
    delete show_caption_property_;
  }

  void AnglesDisplay::onInitialize()
  {
    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "AnglesDisplayObject" << count++;
    overlay_.reset(new OverlayObject(ss.str()));
    onEnable();
    updateSize();
    updateLeft();
    updateTop();
    updateFGColor();
    updateBGColor();
    updateFGAlpha();
    updateFGAlpha2();
    updateBGAlpha();
    updateMinValue();
    updateMaxValue();
    updateTextSize();
    updateShowCaption();
    updateAutoColorChange();
    updateMaxColor();
    updateClockwiseRotate();
    overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
    overlay_->hide();
  }

  void AnglesDisplay::update(float wall_dt, float ros_dt)
  {
    if (update_required_) {
      update_required_ = false;
      overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
      overlay_->setPosition(left_, top_);
      overlay_->setDimensions(overlay_->getTextureWidth(),
                              overlay_->getTextureHeight());
      //drawPlot(sagittal_data_, axial_data_);
      drawPlot(axial_data_, sagittal_data_);
    }
  }
  
  void AnglesDisplay::processSagittalMessage(const std_msgs::Float32::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!overlay_->isVisible()) {
      return;
    }
    if (sagittal_data_ != msg->data || sagittal_first_time_) {
      sagittal_first_time_ = false;
      sagittal_data_ = msg->data;
      if(fabs(sagittal_data_>90))
        sagittal_data_ = -sagittal_data_*fmod(msg->data, 90)/fabs(sagittal_data_);
      update_required_ = true;
    }
  }
  
  void AnglesDisplay::processAxialMessage(const std_msgs::Float32::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!overlay_->isVisible()) {
      return;
    }
    if (axial_data_ != msg->data || axial_first_time_) {
      axial_first_time_ = false;
      axial_data_ = msg->data;
      if(fabs(axial_data_>90))
        axial_data_ = -axial_data_*fmod(msg->data, 90)/fabs(axial_data_);
      update_required_ = true;
    }
  }

  void AnglesDisplay::processTargetMessage(const std_msgs::Bool::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!overlay_->isVisible()) {
      return;
    }
    if(!target_active_ && msg->data)
    {
      target_active_ = true;
      sagittal_target_ = sagittal_data_;
      axial_target_ = axial_data_;
    }
    else if(target_active_ && !msg->data)
    {
      target_active_ = false;
    }
  }

  void AnglesDisplay::drawPlot(double axial_val, double sagittal_val)
  {
    QColor fg_color(fg_color_);

    if (auto_color_change_) {
      double r
        = std::min(1.0, fabs((sagittal_val - min_value_) / (max_value_ - min_value_)));
      if (r > 0.6) {
        double r2 = (r - 0.6) / 0.4;
        fg_color.setRed((max_color_.red() - fg_color_.red()) * r2
                        + fg_color_.red());
        fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2
                          + fg_color_.green());
        fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2
                         + fg_color_.blue());
      }
    }

    
    QColor fg_color2(fg_color);
    QColor bg_color(bg_color_);
    fg_color.setAlpha(fg_alpha_);
    fg_color2.setAlpha(fg_alpha2_);
    bg_color.setAlpha(bg_alpha_);
    int width = overlay_->getTextureWidth();
    int height = overlay_->getTextureHeight();
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_, bg_color);
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);

      const int outer_line_width = 5;
      const int value_line_width = 10;
      const int value_indicator_line_width = 2;
      const int value_padding = 5;
      const int circle_size = 14;
      const int center_size = 8;

      const int value_aabb_offset
        = outer_line_width + value_padding + value_line_width / 2;
      
      painter.setPen(QPen(fg_color, outer_line_width, Qt::SolidLine));

      //draw outter circle
      painter.drawEllipse(outer_line_width / 2, outer_line_width / 2,
                          width - outer_line_width ,
                          height - outer_line_width - caption_offset_);

      //draw center
      painter.setBrush(QBrush(Qt::red));
      painter.setPen(Qt::red);
      
      painter.drawEllipse(outer_line_width/2+((width/2)-center_size/2), 
      outer_line_width/2+(((height-caption_offset_)/2)-center_size/2),
                          center_size,
                          center_size);

      //draw position
      painter.drawEllipse(outer_line_width/2+((width/2)-circle_size/2)+axial_val*width/360, 
      outer_line_width/2+(((height-caption_offset_)/2)-circle_size/2)+sagittal_val*height/360,
                          circle_size,
                          circle_size);

      //draw line
      painter.drawLine(outer_line_width/2+(width/2),
        outer_line_width/2+(((height-caption_offset_)/2)),
        outer_line_width/2+((width/2))+axial_val*width/360, 
        outer_line_width/2+(((height-caption_offset_)/2))+sagittal_val*height/360
      );

      //draw target
      if(target_active_)
      {
        painter.setBrush(QBrush(Qt::green));
        painter.setPen(Qt::green);
        painter.drawEllipse(outer_line_width/2+((width/2)-circle_size/2)+axial_target_*width/360, 
        outer_line_width/2+(((height-caption_offset_)/2)-circle_size/2)+sagittal_target_*height/360,
                            circle_size,
                            circle_size);
      }

      // caption
      if (show_caption_) {
        painter.drawText(0, height - caption_offset_, width, caption_offset_,
                         Qt::AlignCenter | Qt::AlignVCenter,
                         getName());
      }
      
      // done
      painter.end();
      // Unlock the pixel buffer
    }
  }

  
  void AnglesDisplay::subscribe()
  {
    std::string sagittal_topic_name = update_sagittal_topic_property_->getTopicStd();
    std::string axial_topic_name = update_axial_topic_property_->getTopicStd();
    std::string target_topic_name = update_target_topic_property_->getTopicStd();
    if (sagittal_topic_name.length() > 0 && sagittal_topic_name != "/") {
      ros::NodeHandle n;
      sagittal_sub_ = n.subscribe(sagittal_topic_name, 1, &AnglesDisplay::processSagittalMessage, this);
    }
    if (axial_topic_name.length() > 0 && axial_topic_name != "/") {
      ros::NodeHandle n;
      axial_sub_ = n.subscribe(axial_topic_name, 1, &AnglesDisplay::processAxialMessage, this);
    }
    if (target_topic_name.length() > 0 && target_topic_name != "/") {
      ros::NodeHandle n;
      target_sub_ = n.subscribe(target_topic_name, 1, &AnglesDisplay::processTargetMessage, this);
    }
  }

  
  void AnglesDisplay::unsubscribe()
  {
    sagittal_sub_.shutdown();
    axial_sub_.shutdown();
  }

  void AnglesDisplay::onEnable()
  {
    subscribe();
    overlay_->show();
    sagittal_first_time_ = true;
    axial_first_time_ = true;
  }

  void AnglesDisplay::onDisable()
  {
    unsubscribe();
    overlay_->hide();
  }

  void AnglesDisplay::updateSize()
  {
    boost::mutex::scoped_lock lock(mutex_);
    texture_size_ = size_property_->getInt();
    update_required_ = true;
  }
  
  void AnglesDisplay::updateTop()
  {
    top_ = top_property_->getInt();
    update_required_ = true;
  }
  
  void AnglesDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
    update_required_ = true;
  }
  
  void AnglesDisplay::updateBGColor()
  {
    bg_color_ = bg_color_property_->getColor();
    update_required_ = true;

  }

  void AnglesDisplay::updateFGColor()
  {
    fg_color_ = fg_color_property_->getColor();
    update_required_ = true;

  }

  void AnglesDisplay::updateFGAlpha()
  {
    fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
    update_required_ = true;

  }

  void AnglesDisplay::updateFGAlpha2()
  {
    fg_alpha2_ = fg_alpha2_property_->getFloat() * 255.0;
    update_required_ = true;

  }

  
  void AnglesDisplay::updateBGAlpha()
  {
    bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
    update_required_ = true;

  }

  void AnglesDisplay::updateMinValue()
  {
    min_value_ = min_value_property_->getFloat();
    update_required_ = true;

  }

  void AnglesDisplay::updateMaxValue()
  {
    max_value_ = max_value_property_->getFloat();
    update_required_ = true;

  }
  
  void AnglesDisplay::updateTextSize()
  {
    boost::mutex::scoped_lock lock(mutex_);
    text_size_ = text_size_property_->getInt();
    QFont font;
    font.setPointSize(text_size_);
    caption_offset_ = QFontMetrics(font).height();
    update_required_ = true;

  }
  
  void AnglesDisplay::updateShowCaption()
  {
    show_caption_ = show_caption_property_->getBool();
    update_required_ = true;

  }

  
  void AnglesDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }

  void AnglesDisplay::updateAutoColorChange()
  {
    auto_color_change_ = auto_color_change_property_->getBool();
    if (auto_color_change_) {
      max_color_property_->show();
    }
    else {
      max_color_property_->hide();
    }
    update_required_ = true;

  }

  void AnglesDisplay::updateMaxColor()
  {
    max_color_ = max_color_property_->getColor();
    update_required_ = true;

  }

  void AnglesDisplay::updateClockwiseRotate()
  {
    clockwise_rotate_ = clockwise_rotate_property_->getBool();
    update_required_ = true;

  }

   bool AnglesDisplay::isInRegion(int x, int y)
  {
    return (top_ < y && top_ + texture_size_ > y &&
            left_ < x && left_ + texture_size_ > x);
  }

  void AnglesDisplay::movePosition(int x, int y)
  {
    top_ = y;
    left_ = x;
  }

  void AnglesDisplay::setPosition(int x, int y)
  {
    top_property_->setValue(y);
    left_property_->setValue(x);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::AnglesDisplay, rviz::Display )
