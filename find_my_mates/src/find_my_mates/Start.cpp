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

#include "find_my_mates/Start.h"

#include "find_my_mates/Dialog.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mates
{

Start::Start(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{}

void
Start::halt()
{
  ROS_INFO("Start halt");
}

BT::NodeStatus
Start::tick()
{
    ROS_INFO("Start");
    find_my_mates::Dialog forwarder;
    
    forwarder.listen();
    ros::spinOnce();
    if (forwarder.get_start() == 0)
        return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::RUNNING;
}
}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::Start>("Start");
}
