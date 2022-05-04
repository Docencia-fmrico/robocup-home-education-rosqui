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

#include "luggage/Dialog.h"

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

void DetectLuggage::callback_bbx(const sensor_msgs::ImageConstPtr& image,
const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr_color;
  try
  {
      img_ptr_color = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
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

      int x = (box.xmax - box.xmin) / 2;
      int y = (box.ymax - box.ymin) / 2;

      color_[0] = img_ptr_color->image.at<cv::Vec3b>(y,x)[0];
      color_[1] = img_ptr_color->image.at<cv::Vec3b>(y,x)[1];
      color_[2] = img_ptr_color->image.at<cv::Vec3b>(y,x)[2];

      ROS_INFO("MIN_X: %d \t MAX_X: %d\n", min_x, max_x);
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

  ROS_INFO("Detect Luggage Tick");
  luggage::Dialog forwarder;
  ROS_INFO("Speak:");
  ros::Duration(1, 0).sleep();
  forwarder.speak("Good morning, what is your name?");
  ros::Duration(3, 0).sleep();
  ROS_INFO("FIRST LISTEN");
  forwarder.listen(); 
  ros::spinOnce();
  ros::Duration(7, 0).sleep();
  ROS_INFO("SECOND LISTEN");
  forwarder.listen(); 
  ros::spinOnce();

  dialogflow_ros_msgs::DialogflowResult side;
  side = forwarder.getValue();
  for (const auto & param : side.parameters) {
    for (const auto & value : param.value) {
      std::cerr << "\t" << value << std::endl;

      if (value == "left")   // Numero mágico
      {
        ROS_INFO("USER'S LEFT");
        setOutput("color", color_);
        setOutput("bag_pos", "left");
        return BT::NodeStatus::SUCCESS;

      } else if (value == "right")   // Numero mágico
      {
        ROS_INFO("USER'S RIGHT");
        setOutput("color", color_);
        setOutput("bag_pos", "right");
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::DetectLuggage>("DetectLuggage");
}
