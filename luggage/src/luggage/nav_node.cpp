#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class MyNode
{
public:
	MyNode()
	: ac("move_base", true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	void doWork(long int until, std::vector<float> coords)
	{
		for(int i=0; i < 7; i++){
			std::cerr << "coords[" << i << "]: " << coords[i] << std::endl; 
		}
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
					boost::bind(&MyNode::doneCb, this, _1, _2),
					Client::SimpleActiveCallback(),
					boost::bind(&MyNode::feedbackCb, this, _1));

		ROS_INFO("Action sent");

	}

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		//ros::shutdown();
	}

private:
	Client ac;
};