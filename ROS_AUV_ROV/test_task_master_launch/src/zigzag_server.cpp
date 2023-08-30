#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionResult.h>
#include <motion_commons/ForwardActionResult.h>
#include <motion_commons/TurnActionResult.h>
#include <motion_commons/UpwardActionResult.h>
#include <string>

#include <task_actions/zigzagAction.h>

typedef actionlib::SimpleActionServer<task_actions::zigzagAction> server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client_Forward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> Client_Sideward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client_Turn;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client_Upward;
using namespace std;
int count_data;
class TaskZigzagInnerClass
{
private:
	ros::NodeHandle nh_;
	server zigzag_server_;
	string action_name_;
	task_actions::zigzagFeedback feedback_;
	task_actions::zigzagResult result_;
	Client_Forward ForwardClient_;
  	Client_Upward UpwardClient_;
  	Client_Sideward SidewardClient_;
	Client_Turn TurnClient_;
  	motion_commons::ForwardGoal forwardgoal;
  	motion_commons::SidewardGoal sidewardgoal;
  	motion_commons::TurnGoal turngoal;
	motion_commons::UpwardGoal upwardgoal;
	std_msgs::Float64 data_count;
	bool success;
public:
	TaskZigzagInnerClass(string name, string node, string node1, string node2, string node3)
	: zigzag_server_(nh_, name, boost::bind(&TaskZigzagInnerClass::analysisCb, this, _1), false)
	,ForwardClient_(node)
	,SidewardClient_(node1)
	,TurnClient_(node2)
	,UpwardClient_(node3)
	{
		ROS_INFO("Inside constructor");
		zigzag_server_.registerPreemptCallback(boost::bind(&TaskZigzagInnerClass::PreemptCb,this));
		zigzag_server_.start();
	}

	~TaskZigzagInnerClass()
	{}

	void PreemptCb(void)
	{
		ROS_INFO("%s Called when preempted", action_name_.c_str());
	}

	void analysisCb(const task_actions::zigzagGoalConstPtr goal)
	{
		ROS_INFO("Inside analysisCb");
		ros::Rate looprate(12);
		if(!zigzag_server_.isActive())
			return;

		ROS_INFO("%s waiting for motion servers to start");
		ForwardClient_.waitForServer();
		SidewardClient_.waitForServer();
		UpwardClient_.waitForServer();
		TurnClient_.waitForServer();

		upwardgoal.Goal = 50;
		upwardgoal.loop = 100000;
		UpwardClient_.sendGoal(upwardgoal);

		for(int i; i<1000000; i++ )
		{
			for(int i; i<1000; i++)
			{
				forwardgoal.Goal = 100;
				forwardgoal.loop = 10;
				ForwardClient_.sendGoal(forwardgoal);
			}
			ForwardClient_.cancelGoal();
			for(int i; i<1000; i++)
			{
				sidewardgoal.Goal = -50;
				sidewardgoal.loop = 10;
				SidewardClient_.sendGoal(sidewardgoal);
			}
			SidewardClient_.cancelGoal();

			for(int i; i<1000; i++)
			{
				forwardgoal.Goal = 100;
				forwardgoal.loop = 10;
				ForwardClient_.sendGoal(forwardgoal);
			}
			ForwardClient_.cancelGoal();
			
			for(int i; i<1000; i++)
			{
				sidewardgoal.Goal = 50;
				sidewardgoal.loop = 10;
				SidewardClient_.sendGoal(sidewardgoal);
			}
			SidewardClient_.cancelGoal();
			count_data = count_data + 1;
			feedback_.count = count_data;
			zigzag_server_.publishFeedback(feedback_);
		}

		UpwardClient_.cancelGoal();
		success = true;
		result_.MotionCompleted = true;
    	ROS_INFO("%s: Success is %s", action_name_.c_str(), success ? "true" : "false");
		zigzag_server_.setSucceeded(result_);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "zigzag_server");
	ROS_INFO("waiting for Goal");

	TaskZigzagInnerClass taskobject(ros::this_node::getName(), "forward", "sideward", "turnXY", "upward");
	ros::spin();
	return 0;
}
