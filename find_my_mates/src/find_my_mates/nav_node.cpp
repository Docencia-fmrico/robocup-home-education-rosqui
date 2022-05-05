
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

#include "find_my_mates/nav_node.h"
#include <vector>

namespace find_my_mates
{

Navigation::Navigation() : ac("move_base", true)
{
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

void
Navigation::doWork(long int until, std::vector<float> coords)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = coords[0];
    goal.target_pose.pose.position.y = coords[1];
    goal.target_pose.pose.position.z = coords[2];
    goal.target_pose.pose.orientation.x = coords[3];
    goal.target_pose.pose.orientation.y = coords[4];
    goal.target_pose.pose.orientation.z = coords[5];
    goal.target_pose.pose.orientation.w = coords[6];

    ROS_INFO("Sending action");
        ac.sendGoal(goal,
            boost::bind(&Navigation::doneCb, this, _1, _2),
            Client::SimpleActiveCallback(),
            boost::bind(&Navigation::feedbackCb, this, _1));

    ROS_INFO("Action sent");
}

void
Navigation::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void
Navigation::doneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

}  // namespace find_my_mates
