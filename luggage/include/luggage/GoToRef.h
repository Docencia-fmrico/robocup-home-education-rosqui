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

#ifndef LUGGAGE_GOTOREF_H
#define LUGGAGE_GOTOREF_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>
#include "NearGo.h"

#include <move_base_msgs/MoveBaseActionResult.h>

#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>

namespace luggage
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
    std::vector<float> coords_ = {0.37, 3.63, 0.0, 0.0, 0.0, -0.91, 0.41};
    bool first_;
    NearGo neargo_;
};

}  // namespace luggage

#endif  // LUGGAGE_GOTOREF_H
