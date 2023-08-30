#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_actions/dropperAction.h>
#include <task_actions/dropperActionResult.h>
#include <task_actions/dropperActionFeedback.h>
#include <task_actions/buoyAction.h>
#include <task_actions/lineAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/SidewardActionResult.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <task_actions/buoyActionFeedback.h>
#include <task_actions/lineActionFeedback.h>
#include <task_actions/buoyActionResult.h>
#include <task_actions/lineActionResult.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/UpwardActionResult.h>
#include <motion_commons/ForwardActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_actions::buoyAction> ClientBuoy;
typedef actionlib::SimpleActionClient<task_actions::lineAction> ClientLine;
typedef actionlib::SimpleActionClient<task_actions::dropperAction> ClientDropper;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> ClientUpward;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> ClientForward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> ClientSideward;

ClientBuoy *ptrClientBuoy;
ClientLine *ptrClientLine;
ClientDropper *ptrClientDropper;
ClientUpward *ptrClientUpward;
ClientForward *ptrClientForward;
ClientSideward *ptrClientSideward;
task_actions::buoyGoal goalBuoy;
task_actions::lineGoal goalLine;
task_actions::dropperGoal goalDropper;
motion_commons::UpwardGoal goalUpward;
motion_commons::ForwardGoal goalForward;
motion_commons::SidewardGoal goalSideward;
ros::Publisher pub_upward;
ros::Publisher pub_forward;

bool successBuoy = false;
bool successLine = false;
bool successDropper = false;
bool successUpward = false;
bool successDownward = false;
bool successSideward = false;
bool TaskCompleted = false;
float present_depth;

void spinThreadBuoy()
{
  ClientBuoy &temp = *ptrClientBuoy;
  temp.waitForResult();
  successBuoy = (*(temp.getResult())).MotionCompleted;
  if (successBuoy)
  {
    ROS_INFO("%s: motion buoy successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s: motion buoy unsuccessful", ros::this_node::getName().c_str());
}

void spinThreadLine()
{
  ClientLine &temp = *ptrClientLine;
  temp.waitForResult();
  successLine = (*(temp.getResult())).MotionCompleted;
  if (successLine)
  {
    ROS_INFO("%s: motion line successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s: motion line unsuccessful", ros::this_node::getName().c_str());
}

void spinThreadUpwardPressure()
{
  ClientUpward &temp = *ptrClientUpward;
  temp.waitForResult();
  successUpward = (*(temp.getResult())).Result;
  if (successUpward)
  {
    ROS_INFO("%s Bot is at desired height.", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_INFO("%s Bot is not at desired height, something went wrong", ros::this_node::getName().c_str());
  }
}

void spinThreadSideward()
{
  ClientSideward &temp = *ptrClientSideward;
  temp.waitForResult();
  successSideward = (*(temp.getResult())).Result;
  if (successSideward)
  {
    ROS_INFO("%s Bot is at desired side.", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_INFO("%s Bot is not at desired side, something went wrong", ros::this_node::getName().c_str());
  }
}

void spinThreadDownwardPressure()
{
  ClientUpward &temp = *ptrClientUpward;
  temp.waitForResult();
  successDownward = (*(temp.getResult())).Result;
  if (successDownward)
  {
    ROS_INFO("%s Bot is at desired height.", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_INFO("%s Bot is not at desired height, something went wrong", ros::this_node::getName().c_str());
  }
}

void spinThreadDropper()
{
  ClientDropper &temp = *ptrClientDropper;
  temp.waitForResult();
  successDropper = (*(temp.getResult())).MotionCompleted;
  if (successDropper)
  {
    ROS_INFO("%s Bot is at desired height.", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_INFO("%s Bot is not at desired height, something went wrong", ros::this_node::getName().c_str());
  }
}

void buoyCB(task_actions::buoyActionFeedback msg)
{
  ROS_INFO("%s: x_coord = %f, y_coord = %f, distance = %f", ros::this_node::getName().c_str(), msg.feedback.x_coord,
           msg.feedback.y_coord, msg.feedback.distance);
}

void dropperCB(task_actions::buoyActionFeedback msg)
{
  ROS_INFO("%s: x_coord = %f, y_coord = %f, distance = %f", ros::this_node::getName().c_str(), msg.feedback.x_coord,
           msg.feedback.y_coord, msg.feedback.distance);
}

void lineCB(task_actions::lineActionFeedback msg)
{
  ROS_INFO("%s: feedback recieved Angle Remaining = %f, x_coord = %f, y_coord = %f", ros::this_node::getName().c_str(),
           msg.feedback.AngleRemaining, msg.feedback.x_coord, msg.feedback.y_coord);
}

void pressureCB(std_msgs::Float64 pressure_sensor_data)
{
  if (successBuoy)
{  present_depth = pressure_sensor_data.data;}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");

  ros::NodeHandle nh;
  // here buoy_server is the name of the node of the actionserver.
  ros::Subscriber sub_buoy = nh.subscribe<task_actions::buoyActionFeedback>("/buoy_server/feedback", 1000, &buoyCB);
  ros::Subscriber sub_line = nh.subscribe<task_actions::lineActionFeedback>("/line_server/feedback", 1000, &lineCB);
  ros::Subscriber sub_upward = nh.subscribe<std_msgs::Float64>("depth", 1000, &pressureCB);
  pub_upward = nh.advertise<std_msgs::Float64>("upward_distance", 1000);
  pub_forward = nh.advertise<std_msgs::Float64>("forward_motion", 1000);

  ClientBuoy buoyClient("buoy_server");
  ptrClientBuoy = &buoyClient;
  ClientLine lineClient("line_server");
  ptrClientLine = &lineClient;
  ClientDropper dropperClient("dropper_server");
  ptrClientDropper = &dropperClient;
  ClientUpward upwardClient("upward");
  ptrClientUpward = &upwardClient;
  ClientForward forwardClient("forward");
  ptrClientForward = &forwardClient;

  ROS_INFO("Waiting for action server to start.");
  lineClient.waitForServer();
  buoyClient.waitForServer();
  upwardClient.waitForServer();
  forwardClient.waitForServer();
  dropperClient.waitForServer();

  //goalUpward.Goal = 60;
 /// goalUpward.loop = 100000;
 // ClientUpward &canUpward = *ptrClientUpward;
  // send goal
 // canUpward.sendGoal(goalUpward);
 // boost::thread spin_thread(&spinThreadUpwardPressure);

  goalLine.order = true;
  ROS_INFO("Action server started, sending goal to line.");

  ClientLine &canLine = *ptrClientLine;
  // send goal
  canLine.sendGoal(goalLine);
  boost::thread spin_thread_line(&spinThreadLine);

  while (!successLine)
  {
    ros::spinOnce();
  }

  goalBuoy.order = true;
  ROS_INFO("Action server started, sending goal to buoy.");

  ClientBuoy &canBuoy = *ptrClientBuoy;
  // send goal
  canBuoy.sendGoal(goalBuoy);
  boost::thread spin_thread_buoy(&spinThreadBuoy);

  while (!successBuoy)
  {
    ros::spinOnce();
  }

  goalUpward.Goal = present_depth - 10;  // add param
  goalUpward.loop = 10;
  ROS_INFO("Action server started, sending goal to upward.");

  ClientUpward &canUpward = *ptrClientUpward;
  // send goal
  canUpward.sendGoal(goalUpward);
  boost::thread spin_thread_upward(&spinThreadUpwardPressure);

  while (!successUpward)
  {
    ros::spinOnce();
  }

  goalForward.Goal = 200;
  goalForward.loop = 10;
  ROS_INFO("Action server started, sending goal to forward.");

  ClientForward &canForward = *ptrClientForward;
  // send goal
  canForward.sendGoal(goalForward);

  //std_msgs::Float64 mock_forward;
  //mock_forward.data = 250;
 // pub_forward.publish(mock_forward);

  sleep(5);  // add param

  canForward.cancelGoal();

  goalUpward.Goal = present_depth + 11;  // add param
  goalUpward.loop = 10;
  ROS_INFO("Sending goal to upward.");

  ClientUpward &canDownward = *ptrClientUpward;
  // send goal
  canDownward.sendGoal(goalUpward);
  boost::thread spin_thread_downward(&spinThreadDownwardPressure);

  while (!successDownward)
  {
    ros::spinOnce();
  }

  successLine = false;
  goalLine.order = true;
  ROS_INFO("Sending goal to line.");

  // send goal
  canLine.sendGoal(goalLine);
  boost::thread spin_thread_line2(&spinThreadLine);

  while (!successLine)
  {
    ros::spinOnce();
  }
//////////////////////////////////
  goalSideward.Goal = 10;
  goalSideward.loop = 30;
  ClientSideward &canSideward = *ptrClientSideward;
  canSideward.sendGoal(goalSideward);
  boost::thread spin_thread_sideward(&spinThreadSideward);
  while(!successSideward)
  {
    ros::spinOnce();
  }

  if(present_depth<50)
	{ present_depth = 50;}
  else if(present_depth>60)
	{ present_depth = 60;}
 ////////////////////////////////////////// 
  successUpward = false;
  goalUpward.Goal = present_depth - 10;  // add param
  goalUpward.loop = 60;
  ROS_INFO("Action server started, sending goal to upward.");

  // send goal
  canUpward.sendGoal(goalUpward);
  boost::thread spin_thread_upward_2(&spinThreadUpwardPressure);

  while (!successUpward)
  {
    ros::spinOnce();
  }
/////////////////////////////
  goalForward.Goal = 250;
  goalForward.loop = 20;
  ROS_INFO("Action server started, sending goal to forward.");

  // send goal
  canForward.sendGoal(goalForward);
  
  sleep(10);  // add param

  canForward.cancelGoal();
/////////////////////////////////////////
  successDownward = false;
  goalUpward.Goal = present_depth + 11;  // add param
  goalUpward.loop = 60;
  ROS_INFO("Sending goal to upward.");

  // send goal
  canDownward.sendGoal(goalUpward);
  boost::thread spin_thread_downward_2(&spinThreadDownwardPressure);

/////////////////////////////////////////////////
  successSideward = false;
  goalSideward.Goal = -30;
  goalSideward.loop = 50;
  canSideward.sendGoal(goalSideward);
  boost::thread spin_thread_sideward_2(&spinThreadSideward);
  while(!successSideward)
  {
    ros::spinOnce();
  }
/////////////////////////////////////////////////
  successLine = false;
  goalLine.order = true;
  ROS_INFO("Action server started, sending goal to line.");

  // send goal
  canLine.sendGoal(goalLine);
  boost::thread spin_thread_line_3(&spinThreadLine);

  while (!successLine)
  {
    ros::spinOnce();
  }

  goalDropper.order = true;
  ROS_INFO("Action server started, sending goal to line.");
  ClientDropper &canDropper = *ptrClientDropper;
  canDropper.sendGoal(goalDropper);
  boost::thread spin_thread_dropper(&spinThreadDropper);

  while(!successDropper)
  {
    ros::spinOnce();
  }
////////////////////////////////////////////
  if(present_depth<60)
	{ present_depth = 60;}
  else if(present_depth>60)
	{ present_depth = 60;}
  successDownward = false;
  goalUpward.Goal = present_depth + 10;  // add param
  goalUpward.loop = 200;
  ROS_INFO("Sending goal to upward.");

  // send goal
  canDownward.sendGoal(goalUpward);
  boost::thread spin_thread_downward_3(&spinThreadDownwardPressure);

  goalForward.Goal = 250;
  goalForward.loop = 70;
  ROS_INFO("Action server started, sending goal to forward.");

  // send goal
  canForward.sendGoal(goalForward);

  while(!successDownward)
  {
    ros::spinOnce();
  }
  
    successDownward = false;
   goalUpward.Goal = 5;  // add param
   goalUpward.loop = 200;
   ROS_INFO("Sending goal to upward.");

   // send goal
   canDownward.sendGoal(goalUpward);
   boost::thread spin_thread_downward_4(&spinThreadDownwardPressure);

ros::spin();
return 0;
}
