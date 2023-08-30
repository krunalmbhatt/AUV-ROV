#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "message_files/imu_msgs.h"
#include "message_files/pwm_msgs.h"
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/SidewardAction.h>
#include <dynamic_reconfigure/server.h>
#include <sideward_motion/pidConfig.h>
#include <string>

using std::string;

typedef actionlib::SimpleActionServer<motion_commons::SidewardAction> Server;  // defining the Client type
float presentSidePosition = 0, previous_error_angle = 0, previous_pitch_angle = 0;
float previousSidePosition = 0;
float finalSidePosition, error, output, pitch_angle, error_angle, desired_angle, maxband = 15,output_angle, minband = 15;
bool initData = false, initData1 = false;
int output_pwm_top, output_pwm_bottom, pwm_angle_bottom, pwm_angle_top;
message_files::pwm_msgs pwm;  // pwm to be send to arduino
std_msgs::UInt16 top;
std_msgs::UInt16 bottom;
// new inner class, to encapsulate the interaction with actionclient
class innerActionClass
{
private:
  ros::NodeHandle nh_;
  Server sidewardServer_;
  std::string action_name_;
  motion_commons::SidewardFeedback feedback_;
  motion_commons::SidewardResult result_;
  ros::Publisher pwm_top;
  ros::Publisher pwm_bottom;
  float p=2, i=0, d=0, band;

public:
  // Constructor, called when new instance of class declared
  explicit innerActionClass(std::string name)
    :  // Defining the server, third argument is optional
    sidewardServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false)
    , action_name_(name)
  {
    // Add preempt callback
    sidewardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
    // Declaring publisher for PWM
    pwm_top = nh_.advertise<std_msgs::UInt16>("top_pwm", 1000);
    pwm_bottom = nh_.advertise<std_msgs::UInt16>("bottom_pwm", 1000);
    // Starting new Action Server
    sidewardServer_.start();
  }

  // default contructor
  ~innerActionClass(void)
  {
  }

  // callback for goal cancelled ,Stop the bot
  void preemptCB(void)
  {
    top.data = 1500;
    bottom.data = 1500;
    pwm_top.publish(top);
    pwm_bottom.publish(bottom);
    //ROS_INFO("%s pwm send to arduino %d", ros::this_node::getName().c_str(), pwm.data);
    // this command cancels the previous goal
    sidewardServer_.setPreempted();
  }

  // called when new goal recieved; start motion and finish it, if not interupted
  void analysisCB(const motion_commons::SidewardGoalConstPtr goal)
  {
    ROS_INFO("%s Inside analysisCB", ros::this_node::getName().c_str());

    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

 /*   // waiting till we recieve the first value from Camera else it's useless to any calculations
    while (!initData)
    {
      ROS_INFO("%s Waiting to get first input from Camera", ros::this_node::getName().c_str());
      loop_rate.sleep();
    }*/

    finalSidePosition = goal->Goal;

    float derivative = 0, integral = 0, dt = 1.0 / loopRate, derivative_angle = 0, integral_angle = 0;
    bool reached = false;
    top.data = 1500;
    bottom.data = 1500;

    if (!sidewardServer_.isActive())
      return;

    while (!sidewardServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
    {
      error = finalSidePosition - presentSidePosition;
      integral += (error * dt);
      derivative = (presentSidePosition - previousSidePosition) / dt;
      output = (p * error) + (i * integral) + (d * derivative);
      output_pwm_top = 1500 + output;
      output_pwm_bottom = 1500 + output;

      //////angle stabilization/////////
      error_angle = pitch_angle - desired_angle;
      derivative_angle = 2.05*((error_angle - previous_error_angle)/dt)*1000;
         
      if(pitch_angle>5 && pitch_angle<355 )
      { 
        if(error_angle >= previous_error_angle)
        {
          integral_angle += (0.005*error_angle)+2;
          derivative_angle = 0;
        }
      }
      else
        integral_angle = 0;

      if(pitch_angle>180 && pitch_angle<358)
        output_angle =-50+(-3*(360-error_angle))-(integral_angle+derivative_angle);

      else if(pitch_angle<180 && pitch_angle>2)
        output_angle = 50+(3*error_angle)+integral_angle+derivative_angle;
      
      else if(pitch_angle < 5 || pitch_angle > 355)
        output_angle = 0;
      ///////////////////////////////////////////////


      pwm_angle_top = output_pwm_top + output_angle;
      pwm_angle_bottom = output_pwm_bottom - output_angle;
    
      bottom.data = pwm_angle_bottom;
      top.data = pwm_angle_top;

      if(bottom.data > 1900)
      { bottom.data = 1900;}
      else if(bottom.data <1100)
      { bottom.data = 1100;}
      
      if(top.data > 1900)
      { top.data = 1900;}
      else if(top.data <1100)
      { top.data = 1100;}


      if (error <= maxband && error >= minband)
      {
        reached = true;
        top.data = 1500;
        bottom.data = 1500;
        pwm_top.publish(top);
        pwm_bottom.publish(bottom);
        ROS_INFO("%s thrusters stopped", ros::this_node::getName().c_str());
        count++;
      }
      else
      {
        reached = false;
        count = 0;
      }

      if (sidewardServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        sidewardServer_.setPreempted();
        reached = false;
        break;
      }

      feedback_.DistanceRemaining = error;
      sidewardServer_.publishFeedback(feedback_);
      pwm_top.publish(top);
      pwm_bottom.publish(bottom);
      
      ROS_INFO("%s pwm send to forward top %d", ros::this_node::getName().c_str(), top.data);
      ROS_INFO("%s pwm send to forward bottom %d", ros::this_node::getName().c_str(), bottom.data);
      //ROS_INFO("%s pwm send to arduino sideward %d", ros::this_node::getName().c_str(), pwm.data);

      ros::spinOnce();
      loop_rate.sleep();

      previous_error_angle = error_angle;
      
    }
    if (reached)
    {
      result_.Result = reached;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      sidewardServer_.setSucceeded(result_);
    }
  }

  void setPID(float new_p, float new_i, float new_d)
  {
    p = new_p;
    i = new_i;
    d = new_d;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(sideward_motion::pidConfig &config, double level)
{
  ROS_INFO("%s SidewardServer: Reconfigure Request: p= %f i= %f d=%f error band=%f", ros::this_node::getName().c_str(),
           config.p, config.i, config.d);
  object->setPID(config.p, config.i, config.d);
}

void rollCb(message_files::imu_msgs msg)
{
  // this is used to set the final angle after getting the value of first intial position
    previous_pitch_angle = pitch_angle;
    pitch_angle = msg.roll;
}

void distanceCb(std_msgs::Float64 msg)
{
  // this is used to set the final angle after getting the value of first intial position
  if (initData1 == false)
  {
    presentSidePosition = msg.data;
    previousSidePosition = presentSidePosition;
    initData = true;
  }
  else 
  {
    previousSidePosition = presentSidePosition;
    presentSidePosition = msg.data;
  }
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sideward");
  ros::NodeHandle n;
  double p_param, i_param, d_param;
  n.getParam("sideward/p_param", p_param);
  n.getParam("sideward/i_param", i_param);
  n.getParam("sideward/d_param", d_param);

  ros::Subscriber yDistance = n.subscribe<std_msgs::Float64>("side_motion", 1000, &distanceCb);
  ros::Subscriber sub = n.subscribe<message_files::imu_msgs>("imu_data", 1000, &rollCb);
  ROS_INFO("%s Waiting for Goal", ros::this_node::getName().c_str());
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<sideward_motion::pidConfig> server;
  dynamic_reconfigure::Server<sideward_motion::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  sideward_motion::pidConfig config;
  config.p = p_param;
  config.i = i_param;
  config.d = d_param;
  callback(config, 0);

  ros::spin();
  return 0;
}
