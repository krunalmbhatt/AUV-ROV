#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_actions/zigzagAction.h>
#include <task_actions/zigzagActionFeedback.h>
#include <task_actions/zigzagActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_actions::zigzagAction> Client;

Client *ptrClient;
task_actions::zigzagGoal goal;

bool success = false;

void spinThread()
{
	Client &temp = *ptrClient;
  	temp.waitForResult();
  	success = (*(temp.getResult())).MotionCompleted;
  	if (success)
  		ROS_INFO("%s motion successful", ros::this_node::getName().c_str());
  	else
    	ROS_INFO("%s motion unsuccessful", ros::this_node::getName().c_str());
	ros::shutdown();
}

void feedback(task_actions::zigzagActionFeedback msg)
{
  ROS_INFO("%s: count= %f ", ros::this_node::getName().c_str(), msg.feedback.count);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"zigzag_client");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<task_actions::zigzagActionFeedback>("/zigzag_server/feedback", 1000, &feedback);

	Client testClient("zigzag_server");
  	ptrClient = &testClient;

  	ROS_INFO("%s: Waiting for action server to start.", ros::this_node::getName().c_str());
  	testClient.waitForServer();
  	goal.start = true;
	ROS_INFO("%s Action server started, sending goal.", ros::this_node::getName().c_str());
	Client &can = *ptrClient;
  	// send goal
  	can.sendGoal(goal);
  	boost::thread spin_thread(&spinThread);
	ros::spin();
	return 0;
}
