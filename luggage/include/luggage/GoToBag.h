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

#ifndef LUGGAGE_GOTOBAG_H
#define LUGGAGE_GOTOBAG_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>

#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>

namespace luggage
{

class GoToBag : public BT::ActionNodeBase
{
  public:
    explicit GoToBag(const std::string& name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("bag_pos"), BT::InputPort<std::vector<int>>("color")};
    }

    void halt();

    BT::NodeStatus tick();

  private:
    ros::Time detected_ts_;
    ros::NodeHandle nh_;
    std::string bag_pos_;
    static constexpr double FORWARD_VEL = 0.2;
    static constexpr double TURNING_VEL_ = 0.3;
    static constexpr double ACTION_TIME_ = 2.0;
    ros::Publisher pub_vel_;
    bool first;
};

}  // namespace luggage

#endif  // LUGGAGE_GOTOBAG_H
