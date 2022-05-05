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

#include "luggage/Start.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace luggage
{

Start::Start(const std::string& name)
: BT::ActionNodeBase(name, {}),
  nh_()
{
  first_ = true;
}

void
Start::halt()
{
  ROS_INFO("Start halt");
}

BT::NodeStatus
Start::tick()
{
    ROS_INFO("Start");
    if (first_)
    {
      forwarder_.listen();
      first_ = false;
    }

    if (forwarder_.get_start() == 0)
        return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::Start>("Start");
}
