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

#include "find_my_mates/AnalyzePerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace find_my_mates
{

AnalyzePerson::AnalyzePerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_(),
  image_depth_sub(nh_, "/camera/depth/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
  int occupied_pos_ = 0;
  detected_ = false;
  sync_bbx.registerCallback(boost::bind(&AnalyzePerson::callback_bbx, this, _1, _2));
}

void AnalyzePerson::callback_bbx(const sensor_msgs::ImageConstPtr& image,
const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  std::cerr << "person on : " << occupied_pos_ << std::endl;
  detected_ = true;
}

void
AnalyzePerson::halt()
{
  ROS_INFO("AnalyzePerson halt");
}

BT::NodeStatus
AnalyzePerson::tick()
{
    if (detected_)
    {
      setOutput("occupied_pos", occupied_pos_);
      detected_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      occupied_pos_++;
      return BT::NodeStatus::FAILURE;
    }
}
}  // namespace find_my_mates


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::AnalyzePerson>("AnalyzePerson");
}
