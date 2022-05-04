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

#include "luggage/GoToOrigin.h"
#include "nav_node.cpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "move_base_msgs/MoveBaseActionResult.h"

#include "ros/ros.h"

namespace luggage
{

GoToOrigin::GoToOrigin(const std::string& name)
: BT::ActionNodeBase(name, {}),
  nh_()
{
  ROS_INFO("CONSTRUCTOR GoToOrigin");
  result_sub_ = nh_.subscribe("/move_base/result", 1, &GoToOrigin::ResultCallback, this);
  result_ = 0;
  first_ = true;
}

void
GoToOrigin::halt()
{
  ROS_INFO("GoToOrigin halt");
}

void
GoToOrigin::ResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
	result_ = msg->status.status;
}

BT::NodeStatus
GoToOrigin::tick()
{	
	if(first_){
		Navigation my_node_;
		my_node_.doWork(200, coords_);
		first_ = false;
	}

	if (result_ != 0)
		ROS_INFO("Result: %d", result_);

	if (result_ == 3)
	{
		ROS_INFO("LEAVING");
		return BT::NodeStatus::SUCCESS;
	}

  	return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::GoToOrigin>("GoToOrigin");
}
