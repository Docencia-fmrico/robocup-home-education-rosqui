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

#include <vector>

namespace luggage
{

GoToBag::GoToBag(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{
  ROS_INFO("CONSTRUCTOR BAG");
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  first_ = true;
}

void
GoToBag::halt()
{
  ROS_INFO("GoToBag halt");
}

BT::NodeStatus
GoToBag::tick()
{
  if (first_)
  {
    detected_ts_ = ros::Time::now();
    bag_pos_ = getInput<std::string>("bag_pos").value();
    first_ = false;
  }

  std::vector<int> color = getInput<std::vector<int>>("color").value();
  ROS_INFO("R_%d,G:%d,B:%d", color[0], color[1], color[2]);

  geometry_msgs::Twist cmd;
  double current_ts_ = (ros::Time::now() - detected_ts_).toSec();
  ROS_INFO("TIME: %f", current_ts_);

  if ( (current_ts_ < ACTION_TIME_))
  {
      cmd.linear.x = 0;

      if (bag_pos_ == "left")
        cmd.angular.z = TURNING_VEL_;
      else
        cmd.angular.z = -TURNING_VEL_;

      ROS_INFO("TIME: %f %f", current_ts_, TURNING_VEL_);
  }
  else if (current_ts_ >= 5*ACTION_TIME_)
  {
      ROS_INFO("BAG REACHED");
      return BT::NodeStatus::SUCCESS;
  }
  pub_vel_.publish(cmd);
  return BT::NodeStatus::RUNNING;
}
}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::GoToBag>("GoToBag");
}
