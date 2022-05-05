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

#ifndef FIND_MY_MATES_ANALYZEPERSON_H
#define FIND_MY_MATES_ANALYZEPERSON_H

#include <find_my_mates/DialogInterface.h>
#include <sound_play/SoundRequest.h>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <vector>

namespace ph = std::placeholders;

namespace find_my_mates
{
class AnalyzePerson : public BT::ActionNodeBase
{
  public:
    explicit AnalyzePerson(const std::string& name);

    void halt();

    BT::NodeStatus tick();
    
  private:
    ros::NodeHandle nh_;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_ANALYZEPERSON_H
