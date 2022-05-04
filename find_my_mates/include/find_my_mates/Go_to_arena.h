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
/*
#ifndef FIND_MY_MATES_GO_TO_ARENA_H
#define FIND_MY_MATES_GO_TO_ARENA_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <message_filters/subscriber.h>
#include "find_my_mates/BTNavAction.h"

#include "geometry_msgs/Twist.h"

#include <string>

namespace find_my_mates
{

class GoToArena : public BT::ActionNodeBase
{
  public:
    explicit GoToArena(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt();
    BT::NodeStatus on_tick();
    void on_start();
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    static BT::PortsList providedPorts() {
      return {};
    }

  private:
    int counter_;

};

}  // namespace find_my_mates
*/
#endif  // FIND_MY_MATES_GO_TO_ARENA_H
