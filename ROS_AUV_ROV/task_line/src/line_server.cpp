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
#include <motion_commons/TurnAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionResult.h>
#include <motion_commons/ForwardActionResult.h>
#include <motion_commons/TurnActionResult.h>
#include <string>
#include <task_actions/lineAction.h>

typedef actionlib::SimpleActionServer<task_actions::lineAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client_Forward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> Client_Sideward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client_Turn;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client_Upward;
bool isOrange = false;
class TaskLineInnerClass
{
private:
	ros::NodeHandle nh_;
	Server line_server_;
	std::string action_name_;
	task_actions::lineFeedback feedback_;
	task_actions::lineResult result_;
	ros::Subscriber line_detection_switch;
	ros::Subscriber angle_data;
	ros::Publisher switch_angle_detection;
	ros::Publisher yaw_pub_;
	ros::Subscriber present_X_;
	ros::Subscriber present_Y_;
	Client_Forward ForwardClient_;
	Client_Sideward SidewardClient_;
	Client_Turn TurnClient_;
	Client_Upward UpwardClient_;
	motion_commons::ForwardGoal forwardgoal;
	motion_commons::SidewardGoal sidewardgoal;
	motion_commons::TurnGoal turngoal;
	motion_commons::UpwardGoal upwardgoal;
	ros::Publisher motion_x;
	ros::Publisher motion_y;
	std_msgs::Float64 data_X_;
	std_msgs::Float64 data_Y_;
	std_msgs::Float64 angle_goal;
	std_msgs::Float64 sideward;
	std_msgs::Float64 forward;
	bool  success, FrontCenter, SideCenter, LineAlign;
	float x_distance, y_distance;
public:
	TaskLineInnerClass(std::string name, std::string node, std::string node1, std::string node2, std::string node3)
		:line_server_(nh_, name, boost::bind(&TaskLineInnerClass::analysisCb, this, _1),false)
		,action_name_(name)
		,ForwardClient_(node)
		,TurnClient_(node1)
		,SidewardClient_(node2)
		,UpwardClient_(node3)
		{
			ROS_INFO("inside Line constructor");
			line_server_.registerPreemptCallback(boost::bind(&TaskLineInnerClass::preemptCb, this));
			present_Y_ = nh_.subscribe("y_distance",1000, &TaskLineInnerClass::y_Cb, this);
			present_X_ = nh_.subscribe("x_distance",1000, &TaskLineInnerClass::x_Cb, this);
			line_detection_switch = nh_.subscribe<std_msgs::Bool>("line_detected",1000, &TaskLineInnerClass::linedetection, this);
			motion_x = nh_.advertise<std_msgs::Float64>("side_motion",1000);
			motion_y = nh_.advertise<std_msgs::Float64>("forward_motion",1000);
			switch_angle_detection = nh_.advertise<std_msgs::Bool>("line_detection_switch",1000);
			yaw_pub_ = nh_.advertise<std_msgs::Float64>("desired_angle",1000);
			angle_data = nh_.subscribe<std_msgs::Float64>("line_angle",1000,&TaskLineInnerClass::lineAngleListener, this);
			line_server_.start();
		}

		~TaskLineInnerClass(void)
		{
			ROS_INFO("Chup BC");
		}

		void linedetection(std_msgs::Bool msg)
		{
			if(msg.data)
				isOrange = true;
			else
				isOrange = false;
		}

		void y_Cb(std_msgs::Float64 y_data)
		{
			y_distance = y_data.data;
			forward.data = y_data.data;
			motion_y.publish(forward);
		}

		void x_Cb(std_msgs::Float64 x_data)
		{
			x_distance = x_data.data;
			sideward.data = x_data.data;
			motion_x.publish(sideward);
		}

		void lineAngleListener(std_msgs::Float64 msg)
		{
			std_msgs::Float64 angle;
			angle.data = msg.data;
			angle_goal.data = msg.data;
			yaw_pub_.publish(angle);
		}

		void spinThreadSidewardCamera()
		{
			Client_Sideward &tempSideward = SidewardClient_;
			tempSideward.waitForResult();
			SideCenter = (*(tempSideward.getResult())).Result;
			if(SideCenter)
			{
				ROS_INFO("%s: Bot is at side center", action_name_.c_str());
			}
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
			if(FrontCenter)
			{
				ROS_INFO("%s: Bot is at front center", action_name_.c_str());
			}
			else
			{
				ROS_INFO("%s: Bot is not at front center, something went wrong", action_name_.c_str());
				success = false;
			}
		}

		void spinThreadTurnCamera()
		{
			Client_Turn &tempTurn = TurnClient_;
			tempTurn.waitForResult();
			LineAlign = (*(tempTurn.getResult())).Result;
			if(LineAlign)
			{
				ROS_INFO("%s: Bot is at aligned ", action_name_.c_str());
			}
			else
			{
				ROS_INFO("%s: Bot is at not aligned , something went wrong", action_name_.c_str());
				success = false;
			}
		}

		void preemptCb(void)
		{
			success = false;
			ROS_INFO("%s: Preempted", action_name_.c_str());
		}

		void analysisCb(const task_actions::lineGoalConstPtr goal)
		{
			ROS_INFO("Processing inside analysisCb");
			success = true;
			LineAlign = false;
			FrontCenter = false;
			SideCenter = false;
			ros::Rate looprate(12);

			if(!line_server_.isActive())
				return;
			ROS_INFO("%s Waiting for Forward server to start.", action_name_.c_str());
			ForwardClient_.waitForServer();
			SidewardClient_.waitForServer();
			TurnClient_.waitForServer();

			TaskLineInnerClass::detection_switch_on();
			upwardgoal.Goal = 80;
    			upwardgoal.loop = 1000000;
    			UpwardClient_.sendGoal(upwardgoal);
			//stabilizing yaw
			turngoal.AngleToTurn = 0;
			turngoal.loop = 100;
			TurnClient_.sendGoal(turngoal);
			//moving forward
			forwardgoal.Goal = 100;
			forwardgoal.loop = 10;
			ForwardClient_.sendGoal(forwardgoal);
			ROS_INFO("%s searching for line", action_name_.c_str());

			while(goal->order && success && ros::ok())
			{
				if(line_server_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s: Preempted", action_name_.c_str());
					line_server_.setPreempted();
					success = false;
					break;
				}
				looprate.sleep();

				if(isOrange)
					break;

				feedback_.AngleRemaining = angle_goal.data;
				feedback_.x_coord = x_distance;
				feedback_.y_coord = y_distance;
				line_server_.publishFeedback(feedback_);
				ros::spinOnce();
			}

			ROS_INFO("%s: orange is %s", action_name_.c_str(), isOrange ? "true" : "false");
			ForwardClient_.cancelGoal();

			sidewardgoal.Goal = 0;
			sidewardgoal.loop = 10;
			SidewardClient_.sendGoal(sidewardgoal);
			boost::thread spin_thread_sideward_camera(&TaskLineInnerClass::spinThreadSidewardCamera, this);

			forwardgoal.Goal = 0;
			forwardgoal.loop = 10;
			ForwardClient_.sendGoal(forwardgoal);
			boost::thread spin_thread_forward_camera(&TaskLineInnerClass::spinThreadForwardCamera, this);
			ROS_INFO("%s: center ko bichme laa rahe", action_name_.c_str());

			while(goal->order && success && ros::ok())
			{
				if(line_server_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s: Preempted", action_name_.c_str());
					line_server_.setPreempted();
					success = false;
					break;
				}
				looprate.sleep();
				if(FrontCenter && SideCenter)
				{
					ROS_INFO("%s: Line is in centre", action_name_.c_str());
					break;
				}

				feedback_.AngleRemaining = angle_goal.data;
				feedback_.x_coord = x_distance;
				feedback_.y_coord = y_distance;
				line_server_.publishFeedback(feedback_);
				ros::spinOnce();
			}

			TurnClient_.cancelGoal();

			while(angle_goal.data == 0.0)
			{
				sleep(.01);
			}

			forwardgoal.Goal = 0;
			forwardgoal.loop = 10;
			ForwardClient_.sendGoal(forwardgoal);
			boost::thread spin_thread_turn_camera(&TaskLineInnerClass::spinThreadForwardCamera, this);
			ROS_INFO("%s: Aligning", action_name_.c_str());

			while(goal->order && success && ros::ok())
			{
				if(line_server_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s Preempted", action_name_.c_str());
					line_server_.setPreempted();
					success = false;
					break;
				}
				looprate.sleep();
				if(LineAlign)
				{
					ROS_INFO("%s: line is aligned", action_name_.c_str());
					break;
				}

				feedback_.AngleRemaining = angle_goal.data;
				feedback_.x_coord = x_distance;
				feedback_.y_coord = y_distance;
				line_server_.publishFeedback(feedback_);
				ros::spinOnce();
			}

			TaskLineInnerClass::detection_switch_off();

			result_.MotionCompleted = success;
			ROS_INFO("%s: Success is %s", action_name_.c_str(), success ? "true" : "false");
			line_server_.setSucceeded(result_);
		}

		void detection_switch_on()
 		{
		    std_msgs::Bool msg;
		    msg.data = false;
		    switch_angle_detection.publish(msg);
		}

		void detection_switch_off()
  		{
		    std_msgs::Bool msg;
		    msg.data = true;
		    switch_angle_detection.publish(msg);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_server");
	ROS_INFO("Waiting for Goal");
	TaskLineInnerClass taskLineObject(ros::this_node::getName(),"forward", "turnXY", "sideward", "upward");
	ros::spin();
	return 0;
}
