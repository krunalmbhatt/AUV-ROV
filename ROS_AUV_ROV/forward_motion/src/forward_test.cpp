#include <ros/ros.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/ForwardActionResult.h>
#include <message_files/imu_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <forward_motion/forwardConfig.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client;

Client *clientPointer;
motion_commons::ForwardGoal goal;

ros::Publisher ip_data;
ros::Publisher ip_switch;
bool moving = false;
bool success = false;

void spinThread()
{
	Client &temp = *clientPointer;
	temp.waitForResult();
	success = (*(temp.getResult())).Result;
	if(success)
		ROS_INFO("forward_test motion successfull", ros::this_node::getName().c_str());
	else
		ROS_INFO("forward_test motion failed", ros::this_node::getName().c_str());
}

void callback(forward_motion::forwardConfig &config, double level)
{
	ROS_INFO("forward_test Reconfigure requested: %f %s %d",ros::this_node::getName().c_str(), config.double_param, config.bool_param, config.loop);
	Client &can = *clientPointer;
	if(!config.bool_param)
	{
		if(moving)
		{
			moving = false;
			can.cancelGoal();
			ROS_INFO("forward_test Goal Cancelled", ros::this_node::getName().c_str());
		}
		std_msgs::Bool msg;
		msg.data = true;
		ip_switch.publish(msg);
	}
	else
	{
		if(moving)
		{
			can.cancelGoal();
			ROS_INFO("forward_test Goal Cancelled", ros::this_node::getName().c_str());
		}
		std_msgs::Bool msg;
		msg.data = false;
		ip_switch.publish(msg);
		goal.Goal = 100;
		goal.loop = 100;
		can.sendGoal(goal);
		boost::thread spin_thread(&spinThread);
		ROS_INFO("forward_test Goal send %f loop:%d",ros::this_node::getName().c_str(), goal.Goal, goal.loop);
		moving = true;
	}
}
void forwarCb(motion_commons::ForwardActionFeedback msg)
{
	ROS_INFO("forward_test feedback from server %f distance remaining" /*ros::this_node::getName().c_str()*/, msg.feedback.DistanceRemaining);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "forward_test");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<motion_commons::ForwardActionFeedback>("/forward/feedback",1000,&forwarCb);
	//ros::Subscriber ip_data = nh.subscribe<>
	//ip_data = nh.advertise<message_files::x>("linear_displacement",1000);
	ip_switch = nh.advertise<std_msgs::Bool>("buoy_detection_switch",1000);

	Client forwardTestClient("forward");
	clientPointer = &forwardTestClient;

	ROS_INFO("forward_test Waiting for server to start", ros::this_node::getName().c_str());
	forwardTestClient.waitForServer();

	dynamic_reconfigure::Server<forward_motion::forwardConfig> server;
	dynamic_reconfigure::Server<forward_motion::forwardConfig>::CallbackType f;
	f = boost::bind(&callback,_1,_2);
	server.setCallback(f);
	forward_motion::forwardConfig config;
	config.bool_param = true;
	config.double_param = 100;
  config.loop = 100;
	callback(config,0);

	ros::spin();
	return 0;
}
