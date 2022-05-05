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

#include "find_my_mates/Go_to_arena.h"
#include "find_my_mates/Dialog.h"
#include "nav_node.cpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "move_base_msgs/MoveBaseActionResult.h"

#include "ros/ros.h"


namespace find_my_mates
{

Go_to_arena::Go_to_arena(const std::string& name)
: BT::ActionNodeBase(name, {}),
  nh_()
{
    
  ROS_INFO("CONSTRUCTOR Go_to_arena");
  result_sub_ = nh_.subscribe("/move_base/result", 1, &Go_to_arena::ResultCallback, this);
  result_ = 0;
  coords_[0] = 2.0;
}

void
Go_to_arena::halt()
{
  ROS_INFO("Go_to_arena halt");
}

void
Go_to_arena::ResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
	result_ = msg->status.status;
}

BT::NodeStatus
Go_to_arena::tick()

{
	ROS_INFO("Go_to_arena");
	if (result_ == 3)
	{
		ROS_INFO("LEAVING");
		return BT::NodeStatus::SUCCESS;
	}

  	return BT::NodeStatus::RUNNING;
}
}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::Go_to_arena>("Go_to_arena");
}