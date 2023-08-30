#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_actions/dropperAction.h>
#include <task_actions/dropperActionFeedback.h>
#include <task_actions/dropperActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_actions::dropperAction> Client;

Client *ptrClient;
task_actions::dropperGoal goal;
 
bool success = false;

void spinThread()
{
	Client &temp = *ptrClient;
	temp.waitForResult();
	success = (*(temp.getResult())).MotionCompleted;
	if(success)
		ROS_INFO("%s Motion successfull", ros::this_node::getName().c_str());
	else
		ROS_INFO("%s Motion failed", ros::this_node::getName().c_str());
	ros::shutdown();
}

void feedback(task_actions::dropperActionFeedback msg)
{
	ROS_INFO("%s feedback x_coordinates: %f  y_coordinates: %f", ros::this_node::getName().c_str(), msg.feedback.x_coord, msg.feedback.y_coord);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"dropper_client");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<task_actions::dropperActionFeedback>("/dropper_server/feedback", 1000, &feedback);
	Client testClient("dropper_server");
	ptrClient = &testClient;

	ROS_INFO("%s Waiting for dropper server to start", ros::this_node::getName().c_str());
	testClient.waitForServer();
	goal.order = true;
	ROS_INFO("%s Action server started, sending goal", ros::this_node::getName().c_str());

	Client &can = *ptrClient;
	if(ros::ok())
		goal.order = true;
	else
		goal.order = false;
	ROS_INFO("%s goal sent", goal.order ? "true" : "false");
	can.sendGoal(goal);
	boost::thread spin_thread(&spinThread);
	ros::spin();
	return 0;
}