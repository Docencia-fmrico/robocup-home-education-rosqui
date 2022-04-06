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

#include "luggage/GoToBag.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace luggage
{

GoToBag::GoToBag(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{
  ROS_INFO("CONSTRUCTOR BAG");
}

void
GoToBag::halt()
{
  ROS_INFO("GoToBag halt");
}

BT::NodeStatus
GoToBag::tick()
{
  ROS_INFO("Go To Bag Tick");

    std::string bag_pos = getInput<std::string>("bag_pos").value();

    ROS_INFO("BAG_POS:%s", bag_pos.c_str());

  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::GoToBag>("GoToBag");
}
