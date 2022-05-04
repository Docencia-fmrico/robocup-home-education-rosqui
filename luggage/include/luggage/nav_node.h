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

#ifndef LUGGAGE_NAVIGATION_H
#define LUGGAGE_NAVIGATION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace luggage
{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class Navigation
{
public:

	Navigation();
	void doWork(long int until, std::vector<float> coords);
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
	void doneCb(const actionlib::SimpleClientGoalState& state,
  	const move_base_msgs::MoveBaseResultConstPtr& result);

private:
	Client ac;
};

}  // namespace luggage

#endif  // LUGGAGE_NAVIGATION_H
