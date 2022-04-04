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

DetectLuggage::DetectLuggage(const std::string& name)
: BT::ActionNodeBase(name, {}),
  nh_(),
  image_depth_sub(nh_, "/camera/depth/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
  sync_bbx.registerCallback(boost::bind(&DetectLuggage::callback_bbx, this, _1, _2));
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

  int px = 0;
  int py = 0;
  float dist = 0;
  float prob = 0;
  // Darknet only detects person
  for (const auto & box : boxes->bounding_boxes)
  {
    ROS_INFO("PROB: %f", box.probability);
    if ((box.probability > 0.2) && (box.probability > prob))
    {
      prob = box.probability;
      ROS_INFO("DETECTED TRUE");
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;

      dist = img_ptr_depth->image.at<float>(cv::Point(px, py));  // *0.001f;
      if (isnan(dist))
        dist = 0;

      ROS_INFO("person_x: %d \t person_z: %f\n", px, dist);
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
  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::DetectLuggage>("DetectLuggage");
}
