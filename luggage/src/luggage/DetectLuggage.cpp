// Copyright 2022 ROSqui
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "luggage/DetectLuggage.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace luggage
{

DetectLuggage::DetectLuggage(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_(),
  image_color_sub(nh_, "/camera/rgb/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_color_sub, bbx_sub)
{
  sync_bbx.registerCallback(boost::bind(&DetectLuggage::callback_bbx, this, _1, _2));
  min_x = 100;
  max_x = 100;
}

void DetectLuggage::getPredominantColor(int red, int green, int blue)
{
  if (red <= 100 && blue <= 100 && green <= 100)
  {
    ROS_INFO("BLACK");
  }else if (red >= 200 && blue >= 200 && green >= 200)
  {
    ROS_INFO("WHITE");
  }
  else if (red >= green && red >= blue)
  {
    if ((blue <= green + 60) && ( blue >= green - 60))
    {
      ROS_INFO("RED");
    }else if ((blue <= green + 60) && ( blue >= green - 60))
    {
      ROS_INFO("PINK");
    }else{
      ROS_INFO("YELLOW");
    }
  }else if (green >= red && green >= blue)
  {
    if ((red <= blue + 60) && ( red >= blue - 60))
    {
      ROS_INFO("GREEN");
    }else if (red > blue + 60)
    {
      ROS_INFO("YELLOW");
    }else if(blue > red + 60){
      ROS_INFO("LIGHT BLUE");
    }
  }else if (blue > red && blue > green)
  {
    if((green <= red +60) && (green >= red - 60))
    {
      ROS_INFO("BLUE");
    }else if (green > red + 60)
    {
      ROS_INFO("LIGHT BLUE");
    }else if (blue > green + 60)
    {
      ROS_INFO("PINK");
    }
  }
}

void DetectLuggage::callback_bbx(const sensor_msgs::ImageConstPtr& image,
const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr_color;
  try
  {
      img_ptr_color = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC3);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }

  float prob = 0;
  // Darknet only detects person
  for (const auto & box : boxes->bounding_boxes)
  {
    ROS_INFO("PROB: %f", box.probability);
    if (box.probability > 0.8)
    {
      min_x = box.xmin;
      max_x = box.xmax;
      min_y = box.ymin;
      max_y = box.ymax;


      int length_x = (max_x - min_x);
      int length_y = (max_y - min_y);

      int half_y = length_y / 2;

      int last_pixel_y = max_y - length_y/6;

      int red = 0;
      int green = 0;
      int blue = 0;

      
      for(int x = min_x; x < max_x; x++)
      {
        for(int y = half_y; y <  last_pixel_y; y++)
        {
          red = img_ptr_color->image.at<cv::Vec3b>(y,x)[0] + red;
          green = img_ptr_color->image.at<cv::Vec3b>(y,x)[1] + green;
          blue = img_ptr_color->image.at<cv::Vec3b>(y,x)[2] + blue;
        }
        red = red / (last_pixel_y - half_y);
        green = green / (last_pixel_y - half_y);
        blue = blue / (last_pixel_y - half_y);
      }

      red = red / length_x;
      green = green / length_x;
      blue = blue / length_x;

      color_[0] = red;
      color_[1] = green;
      color_[2] = blue;

      getPredominantColor(color_[0], color_[1], color_[2]); //of the referee

    }
  }
}

void
DetectLuggage::halt()
{
  ROS_INFO("DetectLuggage halt");
}

BT::NodeStatus
DetectLuggage::tick()
{
  /*ROS_INFO("Detect Luggage Tick");

  get_color();
  ROS_INFO("R_%d,G:%d,B:%d",color_[0], color_[1], color_[2]);
  setOutput("color", color_);
  
  setOutput("bag_pos", "right");
  return BT::NodeStatus::SUCCESS; */

  ROS_INFO("DETECT LUGGAGE TICK");

    if (min_x < 50)   // Numero mágico
  {
    ROS_INFO("USER'S LEFT");
    setOutput("color", color_);
    setOutput("bag_pos", "left");
    return BT::NodeStatus::SUCCESS;


  } else if (max_x > 450)   // Numero mágico
  {
    ROS_INFO("USER'S RIGHT");
    setOutput("color", color_);
    setOutput("bag_pos", "right");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::DetectLuggage>("DetectLuggage");
}
