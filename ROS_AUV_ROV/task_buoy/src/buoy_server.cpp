#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <task_actions/buoyAction.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/UpwardActionResult.h>
#include <motion_commons/SidewardActionResult.h>
#include <string>

int count = 0;
typedef actionlib::SimpleActionServer<task_actions::buoyAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> ClientForward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> ClientSideward;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> ClientUpward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> ClientTurn;

class TaskBuoyInnerClass
{
private:
  ros::NodeHandle nh_;
  Server buoy_server_;
  std::string action_name_;
  std_msgs::Float64 data_X_;
  std_msgs::Float64 data_distance_;
  //std_msgs::Int32 min_pwm_data;
  task_actions::buoyFeedback feedback_;
  task_actions::buoyResult result_;
  ros::Subscriber sub_ip_;
  ros::Subscriber depth_sub_;
  ros::Subscriber x_data;
  ros::Subscriber y_data;
  ros::Subscriber yaw_sub_;
  ros::Publisher switch_buoy_detection;
  ros::Publisher yaw_pub_;
  ros::Publisher upward_pwm;
  ros::Publisher present_distance_;
  ros::Subscriber present_X_;
  ros::Subscriber present_Y_;
  ClientForward ForwardClient_;
  ClientSideward SidewardClient_;
  ClientUpward UpwardClient_;
  ClientTurn TurnClient_;
  motion_commons::ForwardGoal forwardgoal;
  motion_commons::SidewardGoal sidewardgoal;
  motion_commons::UpwardGoal upwardgoal;
  motion_commons::TurnGoal turngoal;
  ros::Publisher front;
  ros::Publisher back;
  ros::Publisher motion_y;
  ros::Subscriber detection_started;
  //ros::Publisher motion_z;
  //std_msgs::UInt16 front_pwm;
  //std_msgs::UInt16 back_pwm;
  bool successBuoy, sideCenter, heightCenter, IP_stopped, heightGoal, buoy_detection;
  float present_depth, present_Y_coord;
  int min_pwm;

public:
  TaskBuoyInnerClass(std::string name, std::string node, std::string node1, std::string node2, std::string node3)
    : buoy_server_(nh_, name, boost::bind(&TaskBuoyInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
    , SidewardClient_(node1)
    , UpwardClient_(node2)
    , TurnClient_(node3)
  {
    ROS_INFO("%s inside constructor", action_name_.c_str());
    buoy_server_.registerPreemptCallback(boost::bind(&TaskBuoyInnerClass::preemptCB, this));
	detection_started = nh_.subscribe<std_msgs::Bool>("detection_switch", 1000, &TaskBuoyInnerClass::ipdetection, this);
    switch_buoy_detection = nh_.advertise<std_msgs::Bool>("buoy_detection_switch", 1000);
    present_X_ = nh_.subscribe<std_msgs::Float64>("x_distance", 1000, &TaskBuoyInnerClass::xCb,this);
    //present_Y_ = nh_.subscribe<std_msgs::Float64>("y_distance", 1000, &TaskBuoyInnerClass::yCb,this);
    motion_y = nh_.advertise<std_msgs::Float64>("side_motion", 1000);
   // motion_z = nh_.advertise<std_msgs::Float64>("upward_motion", 1000);
    //present_distance_ = nh_.advertise<std_msgs::Float64>("/varun/motion/x_distance", 1000);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("desired_angle", 1000);
    yaw_sub_ = nh_.subscribe<std_msgs::Float64>("yaw_angle", 1000, &TaskBuoyInnerClass::yawCB, this);
    depth_sub_ = nh_.subscribe<std_msgs::Float64>("depth", 1000, &TaskBuoyInnerClass::depthCB, this);
    buoy_server_.start();
  }

  ~TaskBuoyInnerClass(void)
  {
  }

  void ipdetection(std_msgs::Bool msg)
  {
      if(msg.data)
      {buoy_detection = true;}
      else
	{buoy_detection = false;}
  }

  void yawCB(std_msgs::Float64 msg)
  {
    std_msgs::Float64 motion_data;
    motion_data.data = msg.data;
    yaw_pub_.publish(motion_data);
  }

  void depthCB(std_msgs::Float64 depth_data)
  {
    present_depth = depth_data.data;
  }

  void xCb(std_msgs::Float64 msg)
  {
      std_msgs::Float64 motion;
      motion.data = msg.data;
      motion_y.publish(motion);
  }

  void preemptCB(void)
  {
    // Not actually preempting the goal because Prakhar did it in analysisCB
    successBuoy = false;
    ROS_INFO("Called when preempted from the client");
  }
  void spinThreadUpward()
  {
	ClientUpward &tempUpward = UpwardClient_;
	tempUpward.waitForResult();
	heightCenter = (*(tempUpward.getResult())).Result;
	if (heightCenter)
    	{
      	   ROS_INFO("%s Bot is at height center", action_name_.c_str());
    	}
    	else
    	{
      	   ROS_INFO("%s Bot is not at height center, something went wrong", action_name_.c_str());
    	}
}

  void spinThreadSidewardCamera()
  {
    ClientSideward &tempSideward = SidewardClient_;
    tempSideward.waitForResult();
    sideCenter = (*(tempSideward.getResult())).Result;
    if (sideCenter)
    {
      ROS_INFO("%s Bot is at side center", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s Bot is not at side center, something went wrong", action_name_.c_str());
      successBuoy = false;
    }
  }

  void analysisCB(const task_actions::buoyGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");
    sideCenter = false;
    successBuoy = true;
    buoy_detection = false;
    heightGoal = false;
    //min_pwm = 4;
    ros::Rate looprate(12);
    if (!buoy_server_.isActive())
      return;

    ROS_INFO("%s Waiting for Forward server to start.", action_name_.c_str());
    ForwardClient_.waitForServer();
    SidewardClient_.waitForServer();
    UpwardClient_.waitForServer();
    TurnClient_.waitForServer();

    TaskBuoyInnerClass::startBuoyDetection();

    upwardgoal.Goal = 80;
    upwardgoal.loop = 100;
    UpwardClient_.sendGoal(upwardgoal);
	sleep(5);
    // Stabilization of yaw
    turngoal.AngleToTurn = 0;
    turngoal.loop = 100;
    TurnClient_.sendGoal(turngoal);
    
    upwardgoal.Goal = 20;
    upwardgoal.loop = 100;
    UpwardClient_.sendGoal(upwardgoal);
    
    while(goal->order && successBuoy)
    {
	if(buoy_server_.isPreemptRequested() || !ros::ok())
	{
	   ROS_INFO("%s: Preempted", action_name_.c_str());
	   buoy_server_.setPreempted();
	   successBuoy = false;
	   break;
	}
	
	looprate.sleep();
	if(buoy_detection)
	{
	  upwardgoal.Goal = present_depth;
	  break;
	}
    }

    upwardgoal.loop = 100000;
    UpwardClient_.sendGoal(upwardgoal);
    
    sidewardgoal.Goal = 0;
    sidewardgoal.loop = 10;
    SidewardClient_.sendGoal(sidewardgoal);
    boost::thread spin_thread_sideward_camera(&TaskBuoyInnerClass::spinThreadSidewardCamera, this);

    while (goal->order && successBuoy)
    {
      if (buoy_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        buoy_server_.setPreempted();
        successBuoy = false;
        break;
      }
      looprate.sleep();
      if (sideCenter)
      {
        break;
      }
      // publish the feedback
      feedback_.x_coord = data_X_.data;
      feedback_.y_coord = present_Y_coord;
      feedback_.distance = data_distance_.data;
      buoy_server_.publishFeedback(feedback_);
      ros::spinOnce();
    }

    ROS_INFO("%s: Bot is in center of buoy", action_name_.c_str());
    forwardgoal.Goal = 150;
    forwardgoal.loop = 10;
    ForwardClient_.sendGoal(forwardgoal);
    ROS_INFO("%s: Bot is moving forward to hit the buoy", action_name_.c_str());

    while (goal->order && successBuoy)
    {
      if (buoy_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        buoy_server_.setPreempted();
        successBuoy = false;
        break;
      }
      looprate.sleep();

      if (count < 20)
      {
	count = count + 2;
        ROS_INFO("%s: Waiting for hitting the buoy...", action_name_.c_str());
        // sleep(1);
        break;
      }

      feedback_.x_coord = data_X_.data;
      feedback_.y_coord = present_Y_coord;
      feedback_.distance = data_distance_.data;
      buoy_server_.publishFeedback(feedback_);
      ros::spinOnce();
    }

    ForwardClient_.cancelGoal();  // stop motion here

    result_.MotionCompleted = successBuoy;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    buoy_server_.setSucceeded(result_);
  }

  void startBuoyDetection()
  {
    std_msgs::Bool msg;
    msg.data = true;
    switch_buoy_detection.publish(msg);
  }

  void stopBuoyDetection()
  {
    std_msgs::Bool msg;
    msg.data = false;
    switch_buoy_detection.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "buoy_server");
  ROS_INFO("Waiting for Goal");
  TaskBuoyInnerClass taskBuoyObject(ros::this_node::getName(), "forward", "sideward", "upward", "turningXY");
  ros::spin();
  return 0;
}
