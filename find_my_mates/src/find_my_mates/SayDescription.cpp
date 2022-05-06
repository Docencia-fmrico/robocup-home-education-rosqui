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

#include "find_my_mates/SayDescription.h"
#include "nav_node.cpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "move_base_msgs/MoveBaseActionResult.h"

#include "ros/ros.h"

namespace find_my_mates
{

SayDescription::SayDescription(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{
  ROS_INFO("CONSTRUCTOR SayDescription");
  first_ = true;
}

void
SayDescription::halt()
{
  ROS_INFO("SayDescription halt");
}

BT::NodeStatus
SayDescription::tick()
{	

  if(first_){
    detected_ts_ = ros::Time::now();
    pos_ = getInput<int>("occupied_pos").value();
    first_ = false;
  }

  double current_ts_ = (ros::Time::now() - detected_ts_).toSec();

  if(current_ts_< TIME_TO_SPEAK){
    forwarder_.speak("There is a person in position" + std::to_string(pos_));
    return BT::NodeStatus::RUNNING;
  }
  else {
    first_ = true;
    return BT::NodeStatus::SUCCESS;
  }
}
}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::SayDescription>("SayDescription");
}
