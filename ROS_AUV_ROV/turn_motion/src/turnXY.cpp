// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "message_files/imu_msgs.h"
#include "message_files/pwm_msgs.h"
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/TurnAction.h>
#include <dynamic_reconfigure/server.h>
#include <turn_motion/pidConfig.h>
#include <string>
using std::string;

typedef actionlib::SimpleActionServer<motion_commons::TurnAction> Server;
float presentAngularPosition = 0;
float previousAngularPosition = 0;
float finalAngularPosition, error, output, error_val, previous_error, maxband = 15, minband = 15;
int output_pwm_right, output_pwm_left;
bool initData = false;
message_files::pwm_msgs pwm;  // pwm to be send to arduino
std_msgs::UInt16 left;
std_msgs::UInt16 right;
// new inner class, to encapsulate the interaction with actionclient
class innerActionClass
{
private:
  ros::NodeHandle nh_;
  Server turnServer_;
  std::string action_name_;
  motion_commons::TurnFeedback feedback_;
  motion_commons::TurnResult result_;
  ros::Publisher pwm_left;
  ros::Publisher pwm_right;
  float p, i, d, band, p_stablize, i_stablize, d_stablize, p_turn, i_turn, d_turn;

public:
  // Constructor, called when new instance of class declared
  // Defining the server, third argument is optional

  explicit innerActionClass(std::string name):turnServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false), action_name_(name)
  {
    // Add preempt callback
    turnServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
    // Declaring publisher for PWM
    pwm_right = nh_.advertise<std_msgs::UInt16>("right_pwm",1000);
    pwm_left = nh_.advertise<std_msgs::UInt16>("left_pwm", 1000);    // Starting new Action Server
    turnServer_.start();
  }

  // default contructor
  ~innerActionClass(void)
  {
  }

  // callback for goal cancelled, stop the bot
  void preemptCB(void)
  {
    left.data = 1500;
    right.data = 1500;
    pwm_left.publish(left);
    pwm_right.publish(right);
   // ROS_INFO("%s pwm send to arduino %d", ros::this_node::getName().c_str(), pwm.data);
    // this command cancels the previous goal
    turnServer_.setPreempted();
  }


		
  // called when new goal recieved; start motion and finish it, if not
  // interupted
  void analysisCB(const motion_commons::TurnGoalConstPtr goal)
  {
    ROS_INFO("%s Inside analysisCB, Goal = %f", ros::this_node::getName().c_str(), goal->AngleToTurn);
    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);
    // waiting till we recieve the first value from IMU else it's useless to any
    // calculations
/*    while (!initData)
    {
      ROS_INFO("%s Waiting to get first input from IMU", ros::this_node::getName().c_str());
      loop_rate.sleep();
      ros::spinOnce();
    }*/

    finalAngularPosition = presentAngularPosition + goal->AngleToTurn;

    float derivative = 0, integral = 0, dt = 1.0 / loopRate;
    bool reached = false;
    left.data= 1500;
    right.data = 1500;

    if (!turnServer_.isActive())
      return;

    while (!turnServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
    {
      if (goal->loop > 10000)
      {
        p = p_stablize;
        i = i_stablize;
        d = d_stablize;
      }

      else
      {
        p = p_turn;
        i = i_turn;
        d = d_turn;
      }

      error = finalAngularPosition - presentAngularPosition;
      derivative = d*((presentAngularPosition-previousAngularPosition)/dt);
      
      if(presentAngularPosition>5 && presentAngularPosition<355 )
      { 
        if(error >= previous_error)
        {
          integral += (i*error)+2;
          derivative = 0;
        }
      }
      else
        integral = 0;

      if(presentAngularPosition>180 && presentAngularPosition<358)
        output =-50+(-p*(360-error))-(integral+derivative);

      else if(presentAngularPosition<180 && presentAngularPosition>2)
        output = 50+(p*error)+integral+derivative;
 
      else if(presentAngularPosition < 5 || presentAngularPosition > 355)
        output = 0;
 
      /*integral += (error * dt);
      derivative = (presentAngularPosition - previousAngularPosition) / dt;
      output = (p * error) + (i * integral) + (d * derivative);*/
      output_pwm_left = 1500 + output;
      output_pwm_right = 1500 + output;

      right.data = output_pwm_right;
      left.data = output_pwm_left;


      if (error < maxband && error > minband)
      {
        reached = true;
        left.data = 1500;
        right.data = 1500;
        pwm_left.publish(left);
        pwm_right.publish(right);
        ROS_INFO("%s thrusters stopped", ros::this_node::getName().c_str());
        count++;
      }
      else
      {
        reached = false;
        count = 0;
      }

      if (turnServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        turnServer_.setPreempted();
        left.data = 1500;
        right.data = 1500;
        pwm_left.publish(left);
        pwm_right.publish(right);
        reached = false;
        break;
      }

      feedback_.AngleRemaining = error;
      turnServer_.publishFeedback(feedback_);
      pwm_left.publish(left);
      pwm_right.publish(right);
      ROS_INFO("%s pwm send to forward left %d", ros::this_node::getName().c_str(), left.data);
 	  ROS_INFO("%s pwm send to forward right %d", ros::this_node::getName().c_str(), right.data);
      ros::spinOnce();
      loop_rate.sleep();
      previous_error = error;
    }
    if (reached)
    {
      result_.Result = reached;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      turnServer_.setSucceeded(result_);
    }
  }


  void setPID(float new_p_stablize, float new_p_turn, float new_i_stablize, float new_i_turn, float new_d_stablize, float new_d_turn)
  {
    p_stablize = new_p_stablize;
    p_turn = new_p_turn;
    i_stablize = new_i_stablize;
    i_turn = new_i_turn;
    d_stablize = new_d_stablize;
    d_turn = new_d_turn;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(turn_motion::pidConfig &config, double level)
{
  ROS_INFO("%s TurnServer: Reconfigure Request: p_stablize=%f p_turn=%f "
           "i_stablize=%f i_turn=%f d_stablize=%f d_turn=%f error band_turn=%f",
           ros::this_node::getName().c_str(), config.p_stablize, config.p_turn, config.i_stablize, config.i_turn, config.d_stablize, config.d_turn);
  object->setPID(config.p_stablize, config.p_turn, config.i_stablize, config.i_turn, config.d_stablize, config.d_turn);
}

void yawCb(message_files::imu_msgs msg)
{
  // this is used to set the final angle after getting the value of first intial
  // position
  if (initData == false)
  {
    presentAngularPosition = msg.yaw;
    previousAngularPosition = presentAngularPosition;
    initData = true;
  }
  else
  {
    previousAngularPosition = presentAngularPosition;
    presentAngularPosition = msg.yaw;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turnXY");
  ros::NodeHandle n;
  double p_stablize, p_turn, i_stablize, i_turn, d_stablize, d_turn, band_stablize, band_turn;
  n.getParam("turningXY/p_stablize", p_stablize);
  n.getParam("turningXY/p_turn", p_turn);
  n.getParam("turningXY/i_stablize", i_stablize);
  n.getParam("turningXY/i_turn", i_turn);
  n.getParam("turningXY/d_stablize", d_stablize);
  n.getParam("turningXY/d_turn", d_turn);
  ros::Subscriber yaw = n.subscribe<message_files::imu_msgs>("imu_data", 1000, &yawCb);

  ROS_INFO("%s Waiting for Goal", ros::this_node::getName().c_str());
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<turn_motion::pidConfig> server;
  dynamic_reconfigure::Server<turn_motion::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  turn_motion::pidConfig config;
  config.p_stablize = p_stablize;
  config.p_turn = p_turn;
  config.i_stablize = i_stablize;
  config.i_turn = i_turn;
  config.d_stablize = d_stablize;
  config.d_turn = d_turn;
  callback(config, 0);

  ros::spin();
  return 0;
}
