#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_actions/lineAction.h>
#include <task_actions/lineActionFeedback.h>
#include <task_actions/lineActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_actions::lineAction> Client;

Client *ptrClient;
task_actions::lineGoal goal;

bool success = false;

void spinThread()
{
	Client &temp = *ptrClient;
	temp.waitForResult();
	success = (*(temp.getResult())).MotionCompleted;
	if(success)
		ROS_INFO("%s motion successfull", ros::this_node::getName().c_str());
	else
		ROS_INFO("%s motion failed", ros::this_node::getName().c_str());
	ros::shutdown();
}

void motion(task_actions::lineActionFeedback msg)
{
	ROS_INFO("%s Angle remaining = %f, x_coordinates = %f, y_coordinates = %f", 
		ros::this_node::getName().c_str(), msg.feedback.AngleRemaining, 
		msg.feedback.x_coord, msg.feedback.y_coord);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_client");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<task_actions::lineActionFeedback>("/line_server/feedback", 1000, &motion);

	Client testClient("line_server");
	ptrClient = &testClient;

	ROS_INFO("%s: Waiting for line action server to start", ros::this_node::getName().c_str());
	testClient.waitForServer();
	goal.order = true;
	ROS_INFO("%s Action started, goal sent", ros::this_node::getName().c_str());

	Client &can = *ptrClient;

	can.sendGoal(goal);
	boost::thread spin_thread(&spinThread);
	ros::spin();
	return 0;
}