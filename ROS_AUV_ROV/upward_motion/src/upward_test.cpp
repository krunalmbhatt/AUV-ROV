#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/UpwardActionResult.h>
#include <actionlib/client/terminal_state.h>
#include <upward_motion/upwardConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <message_files/displacement.h>

typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client;

Client *clientPointer;

motion_commons::UpwardGoal goal;
ros::Publisher depth_pub;
bool moving = false;
bool success = false;

void spinThread()
{
	Client &temp = *clientPointer;
	temp.waitForResult();
	success = (*(temp.getResult())).Result;
	if(success)
	{
		ROS_INFO("%s motion successfull", ros::this_node::getName().c_str());
	}
	else
	{
		ROS_INFO("%s motion failed", ros::this_node::getName().c_str());
	}
}

void callback(upward_motion::upwardConfig &config, double level)
{
	ROS_INFO("%s Reconfiguring ... %f %s %d", ros::this_node::getName().c_str(), config.double_param, config.bool_param ? "True" : "False", config.loop);
	Client &can = *clientPointer;
	if(!config.bool_param)
	{
		if(moving)
		{
			moving = false;
			can.cancelGoal();
			ROS_INFO("%s Goal Cancelled", ros::this_node::getName().c_str());
		}
		std_msgs::Bool msg;
		msg.data = true;
	}
	else
	{
		if(moving)
		{
			can.cancelGoal();
			ROS_INFO("%s Goal Cancelled", ros::this_node::getName().c_str());
		}
		goal.Goal = config.double_param;
		goal.loop = config.loop;
		can.sendGoal(goal);
		boost::thread spin_thread(&spinThread);
		ROS_INFO("%s Goal Sent %f loop %d", ros::this_node::getName().c_str(), goal.Goal, goal.loop);
		moving = true;
	}
}

void depthCb(std_msgs::Float64 msg)
{
	message_files::displacement depth;
	depth.z = msg.data;
	ROS_INFO("current depth : %f", depth.z);
}

void upwardCb(motion_commons::UpwardActionFeedback msg)
{
	ROS_INFO("upward_test feedback from server %f distance remaining" /*ros::this_node::getName().c_str()*/, msg.feedback.DepthRemaining);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "upward_test");
	ros::NodeHandle n;
	// Subscribing to feedback for ActionServer
	ros::Subscriber sub = n.subscribe<motion_commons::UpwardActionFeedback>("/upward/feedback", 1000, &upwardCb);
	depth_pub = n.advertise<std_msgs::Float64>("gate_detection_switch", 1000);
	ros::Subscriber sud_depth = n.subscribe("depth",1000, &depthCb);
	Client upwardTestClient("upward");
	clientPointer = &upwardTestClient;

	ROS_INFO("Waiting for server to start", ros::this_node::getName().c_str());
	upwardTestClient.waitForServer();

	dynamic_reconfigure::Server<upward_motion::upwardConfig> server;
	dynamic_reconfigure::Server<upward_motion::upwardConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	upward_motion::upwardConfig config;
	config.bool_param = true;
	config.double_param = 51;
	callback(config, 0);
	ros::spin();
	return 0;
}
