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

#ifndef LUGGAGE_DETECTLUGGAGE_H
#define LUGGAGE_DETECTLUGGAGE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "luggage/Dialog.h"

#include <string>
#include <vector>

namespace luggage
{

class DetectLuggage : public BT::ActionNodeBase
{
  public:
    explicit DetectLuggage(const std::string& name, const BT::NodeConfiguration & config);
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("bag_pos") };
    }

    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
    ros::Time detected_ts_;
    bool first_;
    luggage::Dialog forwarder_;
};

}  // namespace luggage

#endif  // LUGGAGE_DETECTLUGGAGE_H
