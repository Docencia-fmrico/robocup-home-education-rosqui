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

#include "find_my_mates/Analyze_person.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mates
{

AnalyzePerson::AnalyzePerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), nh_(), 
image_color_sub(nh_, "/camera/rgb/image_raw", 1),
bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
sync_bbx(MySyncPolicy_bbx(10), image_color_sub, bbx_sub)
{
  //sync_bbx.registerCallback(boost::bind(&AnalyzePerson::callback_bbx, this, _1, _2));
  min_x = 100;
  max_x = 100;
  ROS_INFO("Constructor AnalyzePerson");
}

void
AnalyzePerson::halt()
{
  ROS_INFO("AnalyzePerson halt");
}

BT::NodeStatus
AnalyzePerson::tick()
{
  ROS_INFO("AnalyzePerson tick");
  return BT::NodeStatus::RUNNING; 
}
}  // namespace find_my_mates


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::AnalyzePerson>("AnalyzePerson");
}
