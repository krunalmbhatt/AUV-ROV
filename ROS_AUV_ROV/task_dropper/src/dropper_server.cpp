#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionResult.h>
#include <motion_commons/ForwardActionResult.h>
#include <motion_commons/TurnActionResult.h>
#include <motion_commons/UpwardActionResult.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/TurnAction.h>
#include <string>
#include <task_actions/dropperAction.h>

typedef actionlib::SimpleActionServer<task_actions::dropperAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client_Forward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> Client_Sideward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client_Turn;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client_Upward;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client_Upward;

bool success, FrontCenter, SideCenter, rectangleAlign, XAlign, bucket;

class TaskDropperInnerClass
{
private:
	ros::NodeHandle nh_;
	Server dropper_server_;
	std::string action_name_;
	task_actions::dropperFeedback feedback_;
	task_actions::dropperResult result_;
	ros::Publisher rectangle_switch_;
	ros::Publisher X_switch_;
	ros::Publisher drop_marker_rectangle_;
	ros::Publisher drop_marker_X_;
	ros::Subscriber present_x_;
	ros::Subscriber present_Y_;
	ros::Subscriber detection_;
	Client_Upward UpwardClient_;
	Client_Turn TurnClient_;
	Client_Sideward SidewardClient_;
	Client_Forward ForwardClient_;
	motion_commons::ForwardGoal forwardgoal;
	motion_commons::SidewardGoal sidewardgoal;
	motion_commons::TurnGoal turngoal;
	motion_commons::UpwardGoal upwardgoal;
	ros::Subscriber angle_data;
	ros::Publisher yaw_pub_;
	ros::Publisher motion_x;
	ros::Publisher motion_y;
	std_msgs::Float64 data_x_;
	std_msgs::Float64 data_y_;
	std_msgs::Float64 sideward;
	std_msgs::Float64 forward;
	float y_distance;
	float x_distance;
public:
	TaskDropperInnerClass(std::string name, std::string node, std::string node1, std::string node2, std::string node3)
		:dropper_server_(nh_, name, boost::bind(&TaskDropperInnerClass::analysisCb, this, _1),false)
		,action_name_(name)
		,ForwardClient_(node)
		,TurnClient_(node1)
		,SidewardClient_(node2)
		,UpwardClient_(node3)
	{
		ROS_INFO("inside constructor");
		dropper_server_.registerPreemptCallback(boost::bind(&TaskDropperInnerClass::preemptCb, this));
		//publishers
		rectangle_switch_ = nh_.advertise<std_msgs::Bool>("dropper_rectangle_switch",1000);
		X_switch_ = nh_.advertise<std_msgs::Bool>("dropper_X_switch",1000);
		drop_marker_rectangle_ = nh_.advertise<std_msgs::Bool>("servo_rect",1000);
		drop_marker_X_ = nh_.advertise<std_msgs::Bool>("servo_x",1000);
		yaw_pub_ = nh_.advertise<std_msgs::Float64>("desired_angle",1000);
		motion_x = nh_.advertise<std_msgs::Float64>("side_motion",1000);
		motion_y = nh_.advertise<std_msgs::Float64>("forward_motion",1000);
		//subscribers
		present_Y_ = nh_.subscribe("y_distance",1000, &TaskDropperInnerClass::y_Cb, this);
		present_x_ = nh_.subscribe("x_distance",1000, &TaskDropperInnerClass::x_Cb, this);
		detection_ = nh_.subscribe("detected", 1000, &TaskDropperInnerClass::Bucket_detected, this);
		angle_data = nh_.subscribe<std_msgs::Float64>("yaw_angle",1000,&TaskDropperInnerClass::lineAngleListener, this);
		dropper_server_.start();
	}

	~TaskDropperInnerClass()
	{
		ROS_INFO("Chup BC");
	}

	void y_Cb(std_msgs::Float64 msg)
	{
		y_distance = msg.data;
		forward.data = msg.data;
		motion_y.publish(forward);
	}

	void x_Cb(std_msgs::Float64 msg)
	{
		x_distance = msg.data;
		sideward.data = msg.data;
		motion_x.publish(sideward);
	}

	void lineAngleListener(std_msgs::Float64 msg)
	{
		std_msgs::Float64 angle;
		angle.data = msg.data;
		yaw_pub_.publish(angle);
	}

	void Bucket_detected(std_msgs::Bool msg)
	{
		if(msg.data)
			bucket = true;
		else
			bucket = false;
	}

	void spinThreadSidewardCamera()
	{
		Client_Sideward &tempSideward = SidewardClient_;
	    tempSideward.waitForResult();
	    SideCenter = (*(tempSideward.getResult())).Result;
	    if (SideCenter)
	    	ROS_INFO("%s: Bot is at side center", action_name_.c_str());
	 
	    else
	    {
	      ROS_INFO("%s: Bot is not at side center, something went wrong", action_name_.c_str());
	      success = false;
		}
	}

	void spinThreadSidewardCameraX()
	{
		Client_Sideward &tempSideward = SidewardClient_;
	    tempSideward.waitForResult();
	    SideCenter = (*(tempSideward.getResult())).Result;
	    if (SideCenter)
	    	ROS_INFO("%s: Bot is at side center", action_name_.c_str());
	 
	    else
	    {
	      ROS_INFO("%s: Bot is not at side center, something went wrong", action_name_.c_str());
	      success = false;
		}
	}

	void spinThreadForwardCamera()
	{
	    Client_Forward &tempForward = ForwardClient_;
	    tempForward.waitForResult();
	    FrontCenter = (*(tempForward.getResult())).Result;
	    if (FrontCenter)
	    	ROS_INFO("%s Bot is at Front center", action_name_.c_str());

	    else
	    {
	      ROS_INFO("%s: Bot is not at Front center, something went wrong", action_name_.c_str());
	      success = false;
		}
	}

	void spinThreadForwardCameraX()
	{
	    Client_Forward &tempForward = ForwardClient_;
	    tempForward.waitForResult();
	    FrontCenter = (*(tempForward.getResult())).Result;
	    if (FrontCenter)
	    	ROS_INFO("%s Bot is at Front center", action_name_.c_str());

	    else
	    {
	      ROS_INFO("%s: Bot is not at Front center, something went wrong", action_name_.c_str());
	      success = false;
		}
	}
	void spinThreadTurnCamera()
	{
		Client_Turn &tempTurn = TurnClient_;
		tempTurn.waitForResult();
		XAlign = (*(tempTurn.getResult())).Result;
		if (XAlign)
		{
		  ROS_INFO("%s: Bot is aligned", action_name_.c_str());
		}
		else
		{
		  ROS_INFO("%s: Bot is not aligned, something went wrong", action_name_.c_str());
		  success = false;
		}
	}

	void preemptCb(void)
	{
		ROS_INFO("%s: Called when preempted from client", action_name_.c_str());
	}

	void analysisCb(const task_actions::dropperGoalConstPtr &goal)
	{
		ROS_INFO("inside dropper analysisCb");
		success = true;
		bucket = false;
		FrontCenter = false;
		SideCenter = false;
		rectangleAlign = false;
		XAlign = false;

		ros::Rate looprate(12);

		if(!dropper_server_.isActive())
			return;

		ROS_INFO("%s Waiting for motion servers to start",action_name_.c_str());
		ForwardClient_.waitForServer();
		SidewardClient_.waitForServer();
		TurnClient_.waitForServer();

		TaskDropperInnerClass::rectangle_detection_switch_on();
		upwardgoal.Goal = 80;
    		upwardgoal.loop = 1000000;
    		UpwardClient_.sendGoal(upwardgoal);
		//stabilizing yaw
		turngoal.AngleToTurn = 0;
		turngoal.loop = 10;
		TurnClient_.sendGoal(turngoal);
		//start finding dropper
		forwardgoal.Goal = 100;
		forwardgoal.loop = 10;
		ForwardClient_.sendGoal(forwardgoal);
		ROS_INFO("%s: Searching dropper", action_name_.c_str());

		while(goal->order && success)
		{
			if(dropper_server_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s Preempted", action_name_.c_str());
				dropper_server_.setPreempted();
				success = false;
				break;
			}
			looprate.sleep();

			if(bucket)
				break;

			feedback_.x_coord = x_distance;
			feedback_.y_coord = y_distance;
			dropper_server_.publishFeedback(feedback_);
			ros::spinOnce();
		}

		//stop motion
		ForwardClient_.cancelGoal();

		//centralizing
		sidewardgoal.Goal = -20;
		sidewardgoal.loop = 10;
		SidewardClient_.sendGoal(sidewardgoal);
		boost::thread spin_thread_sideward_camera(&TaskDropperInnerClass::spinThreadSidewardCamera, this);

		forwardgoal.Goal = 0;
		forwardgoal.loop = 10;
		ForwardClient_.sendGoal(forwardgoal);
		boost::thread spin_thread_forward_camera(&TaskDropperInnerClass::spinThreadForwardCamera, this);
		ROS_INFO("%s dropper is going to be centralized", action_name_.c_str());

		while(goal->order && success)
		{
			if(dropper_server_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s preempted", action_name_.c_str());
				dropper_server_.setPreempted();
				success = false;
				break;
			}
			looprate.sleep();
			if(FrontCenter && SideCenter)
			{
				ROS_INFO("%s dropper centralized to rectangle", action_name_.c_str());
				std_msgs::Bool msg;
				msg.data = true;
				drop_marker_rectangle_.publish(msg);
				break;
			}

			feedback_.x_coord = x_distance;
			feedback_.y_coord = y_distance;
			dropper_server_.publishFeedback(feedback_);
			ros::spinOnce();
		}



		//ForwardClient_.cancelGoal();
		//SidewardClient_.cancelGoal();
		//TaskDropperInnerClass::x_detection_switch_on();
		
		FrontCenter = false;
		SideCenter = false;
		success = true;
		sidewardgoal.Goal = 20;
		sidewardgoal.loop = 10;
		SidewardClient_.sendGoal(sidewardgoal);
		boost::thread spin_thread_sideward_camera_X(&TaskDropperInnerClass::spinThreadSidewardCamera, this);

		forwardgoal.Goal = 0;
		forwardgoal.loop = 10;
		ForwardClient_.sendGoal(forwardgoal);
		boost::thread spin_thread_forward_camera_X(&TaskDropperInnerClass::spinThreadForwardCamera, this);
		ROS_INFO("%s dropper is going to be centralized X", action_name_.c_str());

		while(goal->order && success)
		{
			if(dropper_server_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s preempted", action_name_.c_str());
				dropper_server_.setPreempted();
				success = false;
				break;
			}
			looprate.sleep();
			if(FrontCenter && SideCenter)
			{
				ROS_INFO("%s dropper centralized to x", action_name_.c_str());
				std_msgs::Bool msg;
				msg.data = true;
				drop_marker_X_.publish(msg);
				break;
			}

			feedback_.x_coord = x_distance;
			feedback_.y_coord = y_distance;
			dropper_server_.publishFeedback(feedback_);
			ros::spinOnce();
		}

		TaskDropperInnerClass::rectangle_detection_switch_off();

		ForwardClient_.cancelGoal();

		result_.MotionCompleted = success;
		ROS_INFO("%s Success is %s",action_name_.c_str(), success ? "true" : "false");
		dropper_server_.setSucceeded(result_);
	}

	void x_detection_switch_on()
	{
		std_msgs::Bool msg;
		msg.data = true;
		X_switch_.publish(msg);
	}

	void x_detection_switch_off()
	{
		std_msgs::Bool msg;
		msg.data = false;
		X_switch_.publish(msg);
	}

	void rectangle_detection_switch_on()
	{
		std_msgs::Bool msg;
		msg.data = true;
		rectangle_switch_.publish(msg);
	}

	void rectangle_detection_switch_off()
	{
		std_msgs::Bool msg;
		msg.data = false;
		rectangle_switch_.publish(msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dropper_server");
	ROS_INFO("Waiting for Goal");
	TaskDropperInnerClass taskdropperObject(ros::this_node::getName(), "forward", "turnXY", "sideward", "upward");
	ros::spin();
	return 0;
}
