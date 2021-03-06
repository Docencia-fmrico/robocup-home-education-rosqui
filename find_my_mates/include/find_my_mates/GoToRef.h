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

#ifndef FIND_MY_MATES_GOTOREF_H
#define FIND_MY_MATES_GOTOREF_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>

#include <move_base_msgs/MoveBaseActionResult.h>

#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>

namespace find_my_mates
{

class GoToRef : public BT::ActionNodeBase
{
  public:
    explicit GoToRef(const std::string& name);
    void halt();
    BT::NodeStatus tick();
    void ResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber result_sub_;
    int result_;
    std::vector<float> coords_ = {4.73, 0.14, 0.0, 0.0, 0.0, 0.02, 0.99};
    bool first_;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_GOTOREF_H
