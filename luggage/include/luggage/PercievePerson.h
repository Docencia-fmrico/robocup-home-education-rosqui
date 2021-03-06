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

#ifndef LUGGAGE_PERCIEVEPERSON_H
#define LUGGAGE_PERCIEVEPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

#include <string>

namespace luggage
{

class PercievePerson : public BT::ActionNodeBase
{
  public:
    explicit PercievePerson(const std::string& name, const BT::NodeConfiguration & config);
    void callback_bbx(const sensor_msgs::ImageConstPtr& image,
    const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    bool compare_colors(int R1, int G1, int B1, int R2, int G2, int B2);
    void halt();

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<double>("person_z"), BT::OutputPort<int>("person_x")};
    }


    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;
    ros::Time initial_ts_;
    bool detected;
    bool first;
};

}  // namespace luggage

#endif  // LUGGAGE_PERCIEVEPERSON_H
