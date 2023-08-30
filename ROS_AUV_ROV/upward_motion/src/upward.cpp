#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "message_files/imu_msgs.h"
#include "message_files/pwm_msgs.h"
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/UpwardAction.h>
#include <dynamic_reconfigure/server.h>
#include <upward_motion/pidConfig.h>
#include <string>

using std::string;

typedef actionlib::SimpleActionServer<motion_commons::UpwardAction> Server;  // defining the Client type
float presentDepth = 0;
float previousDepth = 0, previous_roll_angle = 0;
float finalDepth, error, output, error_angle, roll_angle, minband = 15, maxband = 15, desired_angle =0, previous_error_angle, output_angle;
int output_pwm_front, output_pwm_back, pwm_angle_front, pwm_angle_back;
bool initData = false, initData1 = false;
message_files::pwm_msgs pwm;  // pwm to be send to arduino
std_msgs::UInt16 front;
std_msgs::UInt16 back;
// new inner class, to encapsulate the interaction with actionclient
class innerActionClass
{
private:
  ros::NodeHandle nh_;
  Server upwardServer_;
  std::string action_name_;
  motion_commons::UpwardFeedback feedback_;
  motion_commons::UpwardResult result_;
  ros::Publisher pwm_front;
  ros::Publisher pwm_back;
  float p = 2, i=0, d=0, band, p_stablize, i_stablize, d_stablize, band_stablize, p_upward, i_upward, d_upward, band_upward;
  float maxPwm;

public:
  // Constructor, called when new instance of class declared
  explicit innerActionClass(std::string name)
    :  // Defining the server, third argument is optional
    upwardServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false), action_name_(name)
  {
    // Add preempt callback
    upwardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
    // Declaring publisher for PWM
    pwm_front = nh_.advertise<std_msgs::UInt16>("front_pwm",1000);
    pwm_back = nh_.advertise<std_msgs::UInt16>("back_pwm", 1000);
    // Starting new Action Server
    upwardServer_.start();
  }

  // default contructor
  ~innerActionClass(void)
  {
  }

  // callback for goal cancelled; Stop the bot
  void preemptCB(void)
  {
    front.data = 1500;
    back.data = 1500;
    pwm_front.publish(front);
    pwm_back.publish(back);
    //ROS_INFO("%s pwm send to arduino %d", ros::this_node::getName().c_str(), pwm.data);
    // this command cancels the previous goal
    upwardServer_.setPreempted();
  }

  // called when new goal recieved; Start motion and finish it, if not interrupted
  void analysisCB(const motion_commons::UpwardGoalConstPtr goal)
  {
    ROS_INFO("%s: Inside analysisCB", ros::this_node::getName().c_str());

    int count = 0;
    int loopRate = 2;
    ros::Rate loop_rate(loopRate);

    // waiting till we recieve the first value from Camera/pressure sensor else it's useless do any calculations
    /*while (!initData)
    {
      ROS_INFO("%s: Waiting to get first input at topic zDistance", ros::this_node::getName().c_str());
      loop_rate.sleep();
    }*/

    finalDepth = goal->Goal;

    float derivative = 0, integral = 0, dt = 1.0 / loopRate, integral_angle = 0, derivative_angle = 0;
    bool reached = false;
    front.data = 1500;
    back.data = 1500;

    if (!upwardServer_.isActive())
      return;

    while (!upwardServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
    {
      if (goal->loop > 10000)
      {
        p = p_stablize;
        i = i_stablize;
        d = d_stablize;
      }

      else
      {
        p = p_upward;
        i = i_upward;
        d = d_upward;
      }

      derivative = ((presentDepth-previousDepth)/dt);
      error = finalDepth - presentDepth;
      integral += (error * dt);
      output = (p * error) + (i * integral) + (d * derivative);
      output_pwm_front = 1500 + output;
      output_pwm_back = 1500 + output;
      
     ///////angle stabilization///////
      error_angle = roll_angle - desired_angle;
      derivative_angle = 2.05*((error_angle - previous_error_angle)/dt);

      if(roll_angle>5 && roll_angle<355 )
      { 
        if(error_angle >= previous_error_angle)
        {
          integral_angle += (0.005*error_angle)+2;
          derivative_angle = 0;
        }
      }
      else
        integral_angle = 0;

      if(roll_angle>180 && roll_angle<358)
        output_angle =-50+(-3*(360-error_angle))-(integral_angle+derivative_angle);

      else if(roll_angle<180 && roll_angle>2)
        output_angle = 50+(3*error_angle)+integral_angle+derivative_angle;

      else if(roll_angle < 5 || roll_angle > 355)
        output_angle = 0;
      //////////////////////////////////////

      pwm_angle_front = output_pwm_front + output_angle;
      pwm_angle_back = output_pwm_back - output_angle;

      front.data = pwm_angle_front;
      back.data = pwm_angle_back;
      
      			if(front.data > 1900)
			{	front.data = 1900;}
			else if(front.data <1100)
			{ front.data = 1100;}
			
			if(back.data > 1900)
			{	back.data = 1900;}
			else if(back.data <1100)
			{ back.data = 1100;}


      if (error <= maxband && error >= minband)
      {
        reached = true;
        front.data = 1500;
        back.data = 1500;
        pwm_front.publish(front);
        pwm_back.publish(back);
        ROS_INFO("%s: thrusters stopped", ros::this_node::getName().c_str());
        count++;
      }
      else
      {
        reached = false;
        count = 0;
      }

      if (upwardServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: %s: Preempted", ros::this_node::getName().c_str(), action_name_.c_str());
        // set the action state to preempted
        upwardServer_.setPreempted();
        reached = false;
        break;
      }

      feedback_.DepthRemaining = error;
      upwardServer_.publishFeedback(feedback_);
      pwm_front.publish(front);
      pwm_back.publish(back);
      //ROS_INFO("%s: pwm send to arduino upward %d", ros::this_node::getName().c_str(), pwm.data);
      ROS_INFO("%s pwm send to forward front %d", ros::this_node::getName().c_str(), front.data);
      ROS_INFO("%s pwm send to forward back %d", ros::this_node::getName().c_str(), back.data);
      ros::spinOnce();
      loop_rate.sleep();
    }
    if (reached)
    {
      result_.Result = reached;
      ROS_INFO("%s: %s: Succeeded", ros::this_node::getName().c_str(), action_name_.c_str());
      // set the action state to succeeded
      upwardServer_.setSucceeded(result_);
    }
  }

  void setPID(float new_p_stablize, float new_p_upward, float new_i_stablize, 
              float new_i_upward, float new_d_stablize, float new_d_upward)
  {
    p_stablize = new_p_stablize;
    p_upward = new_p_upward;
    i_stablize = new_i_stablize;
    i_upward = new_i_upward;
    d_stablize = new_d_stablize;
    d_upward = new_d_upward;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(upward_motion::pidConfig &config, double level)
{
  ROS_INFO("%s: UpwardServer: Reconfigure Request: p_stablize=%f p_upward=%f  i_stablize=%f i_upward=%f d_stablize=%f d_upward=%f "
			,ros::this_node::getName().c_str(), config.p_stablize, config.p_upward, config.i_stablize,
           config.i_upward, config.d_stablize, config.d_upward);
  object->setPID(config.p_stablize, config.p_upward, config.i_stablize, config.i_upward, config.d_stablize, config.d_upward);
}

void rollCb(message_files::imu_msgs msg)
{
  // this is used to set the final angle after getting the value of first intial
  // position
 
  previous_roll_angle = roll_angle;
  roll_angle = msg.pitch;
}

void distanceCb(std_msgs::Float64 msg)
{
  // this is used to set the final depth after getting the value of first intial position
  if (initData == false)
  {
    previousDepth = presentDepth;
    presentDepth = msg.data;
    initData = true;
  }
  else
  {
    previousDepth = presentDepth;
    presentDepth = msg.data;
  }
  ROS_INFO("current depth: %f", msg.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "upward");
  ros::NodeHandle n;
  double p_stablize, p_upward, i_stablize, i_upward, d_stablize, d_upward;
  double maxPwm;
  n.getParam("upward/maxPwm", maxPwm);
  n.getParam("upward/p_stablize", p_stablize);
  n.getParam("upward/p_upward", p_upward);
  n.getParam("upward/i_stablize", i_stablize);
  n.getParam("upward/i_upward", i_upward);
  n.getParam("upward/d_stablize", d_stablize);
  n.getParam("upward/d_upward", d_upward);
  ros::Subscriber zDistance = n.subscribe<std_msgs::Float64>("depth", 1000, &distanceCb);
  ros::Subscriber sub = n.subscribe<message_files::imu_msgs>("imu_data", 1000, &rollCb);
  ROS_INFO("%s: Waiting for Goal", ros::this_node::getName().c_str());
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<upward_motion::pidConfig> server;
  dynamic_reconfigure::Server<upward_motion::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  upward_motion::pidConfig config;
  //config.maxPwm = maxPwm;
  config.p_stablize = p_stablize;
  config.p_upward = p_upward;
  config.i_stablize = i_stablize;
  config.i_upward = i_upward;
  config.d_stablize = d_stablize;
  config.d_upward = d_upward;  
  callback(config, 0);

  ros::spin();
  return 0;
}
