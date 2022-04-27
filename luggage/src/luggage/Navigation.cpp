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

#include "luggage/Navigation.h"
#include "nav_node.cpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <move_base_msgs/MoveBaseActionResult.h>

#include "ros/ros.h"

namespace luggage
{

Navigation::Navigation(const std::string& name)
: BT::ActionNodeBase(name, {}),
  nh_()
{
  ROS_INFO("CONSTRUCTOR NAVIGATION");
  result_sub_ = nh_.subscribe("/move_base/result", 1, &Navigation::ResultCallback, this);
  result_ = 0;
}

void
Navigation::halt()
{
  ROS_INFO("Navigation halt");
}

void
Navigation::ResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
	//result_ = msg->status;
	result_ = 3;
}

BT::NodeStatus
Navigation::tick()
{
	MyNode my_node;
	my_node.doWork(200);

	ROS_INFO("NAV START");

	while (result_ != 3)
	{
		ros::spinOnce();
		ROS_INFO("Result: %d",result_);
	}

  	return BT::NodeStatus::SUCCESS;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::Navigation>("Navigation");
}
