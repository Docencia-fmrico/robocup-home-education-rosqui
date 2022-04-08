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
  image_depth_sub(nh_, "/camera/depth/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
  sync_bbx.registerCallback(boost::bind(&DetectLuggage::callback_bbx, this, _1, _2));
  min_x = 100;
  max_x = 100;
}

void DetectLuggage::callback_bbx(const sensor_msgs::ImageConstPtr& image,
const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr_depth;
  try
  {
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
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
  /*sleep(2);
  setOutput("bag_pos", "right");
  return BT::NodeStatus::SUCCESS;*/

  if (min_x < 50)   // Numero mágico
  {
    ROS_INFO("USER'S LEFT");
    setOutput("bag_pos", "left");
    return BT::NodeStatus::SUCCESS;


  } else if (max_x > 450)   // Numero mágico
  {
    ROS_INFO("USER'S RIGHT");
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
