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
// See the License for the specific674b3e6 language governing permissions and
// limitations under the License.

#include "luggage/FollowPerson.h"
#include "luggage/PIDController.h"
#include "geometry_msgs/Point.h"
#include "NearGo.cpp"
#include <string>
#include <termios.h>

namespace luggage
{

FollowPerson::FollowPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), linear_pid_(0.0, 1.0, 0.0, 0.3), angular_pid_(0.0, 1.0, 0.0, 0.5)
{
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
}

void
FollowPerson::halt()
{
  ROS_INFO("FollowPerson halt");
}

char 
FollowPerson::getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

BT::NodeStatus
FollowPerson::tick()
{
    ROS_INFO("FollowPerson tick");

    int c = getch();   // call your non-blocking input function
    if (c == 'a')
      return BT::NodeStatus::SUCCESS;

	NearGo neargo_;
	bool collision = neargo_.scan();
	if (collision)
		return BT::NodeStatus::RUNNING;

    int X = getInput<int>("person_x").value();
    double Z = getInput<double>("person_z").value();

    ROS_INFO("X:%d Z:%lf", X, Z);
    
    geometry_msgs::Twist cmd;
    angular_pid_.set_pid(0.4, 0.05, 0.55);
    linear_pid_.set_pid(0.4, 0.05, 0.55);
    if (X > 320)
    {
      X *= -X;
    }
      cmd.angular.z = angular_pid_.get_output(X);
      cmd.linear.x = linear_pid_.get_output(Z-1);

      ROS_INFO("X: %d = %lf\t Z: %lf = %lf", X, cmd.angular.z, Z-1, cmd.linear.x);
    pub_vel_.publish(cmd);

    return BT::NodeStatus::RUNNING;
}

}  // namespace luggage

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<luggage::FollowPerson>("FollowPerson");
}
