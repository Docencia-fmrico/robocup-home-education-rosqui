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

#ifndef FIND_MY_MATES_SAYDESCRIPTION_H
#define FIND_MY_MATES_SAYDESCRIPTION_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>

#include <move_base_msgs/MoveBaseActionResult.h>

#include "geometry_msgs/Twist.h"

#include "Dialog.h"

#include <string>
#include <vector>

namespace find_my_mates
{

class SayDescription : public BT::ActionNodeBase
{
  public:
    explicit SayDescription(const std::string& name, const BT::NodeConfiguration & config);
    void halt();
    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("occupied_pos") };
    }
  private:
    ros::NodeHandle nh_;
    find_my_mates::Dialog forwarder_;
    ros::Time detected_ts_;
    int TIME_TO_SPEAK = 5;
    int pos_;
    bool first_;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_SAYDESCRIPTION_H
