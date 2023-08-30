#include "ros/ros.h"
#include "message_files/imu_msgs.h"
#include "message_files/displacement.h"
#include <cstdlib>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "message_files/pwm_msgs.h"
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/ForwardAction.h>
#include <dynamic_reconfigure/server.h>
#include <forward_motion/pidConfig.h>
#include <string>

int x=0;
typedef actionlib::SimpleActionServer<motion_commons::ForwardAction> Server;  // defining the Client type
float presentForwardPosition = 0, previous_error_angle = 0, previous_yaw_angle = 0, finalAngularPosition;
float previousForwardPosition = 0;
float finalForwardPosition, error, output ,yaw_angle = 0, error_angle, desired_angle=0, maxband = 15 ,output_angle, minband = 15;
bool initData = false, initData1 = false;
int output_pwm_right = 1500, output_pwm_left = 1500, pwm_angle_right, pwm_angle_left;
message_files::pwm_msgs pwm;
std_msgs::UInt16 left;
std_msgs::UInt16 right;
using std::string;
class innerActionClass
{
private:
	ros::NodeHandle nh_;
	Server forwardServer_;
	std::string action_name_;
	motion_commons::ForwardFeedback feedback_;
	motion_commons::ForwardResult result_;
  	ros::Publisher pwm_left;
  	ros::Publisher pwm_right;
  	ros::Subscriber desired_angle_;
	float kp ,ki ,kd;

public:
	innerActionClass(string name):forwardServer_(nh_,name,boost::bind(&innerActionClass::analysisCb,this,_1),false),action_name_(name)
	{
		forwardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCb,this));
    pwm_right = nh_.advertise<std_msgs::UInt16>("right_pwm",1000);
    pwm_left = nh_.advertise<std_msgs::UInt16>("left_pwm", 1000);
    desired_angle_ = nh_.subscribe<std_msgs::Float64>("desired_angle",1000, &innerActionClass::angleCb, this);
		forwardServer_.start();
	}
	~innerActionClass(void)
	{
	}

	void angleCb(std_msgs::Float64 msg)
	{
		finalAngularPosition = msg.data;
		
	}
	
	void preemptCb(void)
	{
		left.data = 1500;
		right.data = 1500;
		pwm_left.publish(left);
   		pwm_right.publish(right);
		//ROS_INFO("%s pwm send to driver %d"ros::this_node::getName().c_str(), pwm.data);
		forwardServer_.setPreempted();
	}


	void analysisCb(const motion_commons::ForwardGoalConstPtr &goal)
	{
		ROS_INFO("%s Inside analysisCb", ros::this_node::getName().c_str());
		int count = 0;
		int looprate = 2;
		ros::Rate loop_rate(looprate);
		presentForwardPosition = 0;
/*
		while (!initData)
    	{
      		ROS_INFO("%s Waiting to get first input from IMU", ros::this_node::getName().c_str());
      		loop_rate.sleep();
      		ros::spinOnce();
    	}*/

    	finalForwardPosition = goal->Goal;
		float derivative = 0, derivative_angle = 0, integral_angle = 0, integral = 0, dt = 1.0 / looprate;
    	bool reached = false;
		left.data = 1500;
		right.data = 1500;

		
		
		if(!forwardServer_.isActive())
			return;
		while(!forwardServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
		{
			error = finalForwardPosition - presentForwardPosition;
      		integral += (error * dt);
      		derivative = (presentForwardPosition - previousForwardPosition) / dt;
			output = (2 * error) + (0 * integral) + (0 * derivative);
			output_pwm_left = 1500 + output;
			output_pwm_right = 1500 + output;
			//ROS_INFO("stabilizing");
			//ROS_INFO("%f Yaw data", yaw_angle);
			
			/////////angle stabilization////////////
			error_angle = yaw_angle - finalAngularPosition;
			derivative_angle = 2.05*((error_angle - previous_error_angle)/dt);
			ROS_INFO("%f goal", goal->Goal);
			if(yaw_angle>5 && yaw_angle<355 )
			{ 
		    	if(error_angle >= previous_error_angle)
		        {
		          integral_angle += (0.005*error_angle)+2;
		          derivative_angle = 0;
		        }
		    }
		    else
		    	integral_angle = 0;

		    if(yaw_angle>180 && yaw_angle<358)
		    	output_angle =-50+(-3*(360-error_angle))-(integral_angle+derivative_angle);

		    else if(yaw_angle<180 && yaw_angle>2)
		    	output_angle = 50+(3*error_angle)+integral_angle+derivative_angle;

		    else if(yaw_angle < 5 || yaw_angle > 355)
		    	output_angle = 0;
		    
		    ROS_INFO("%f output", output);
		    ////////////////////////////

	      	pwm_angle_left = output_pwm_left + output_angle;
	      	pwm_angle_right = output_pwm_right - output_angle;

	      	left.data = pwm_angle_left;
	      	right.data = pwm_angle_right;
			
			if(left.data > 1900)
			{	left.data = 1900;}
			else if(left.data <1100)
			{ left.data = 1100;}
			
			if(right.data > 1900)
			{	right.data = 1900;}
			else if(right.data <1100)
			{ right.data = 1100;}
			
			if (forwardServer_.isPreemptRequested() || !ros::ok())
      		{
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		// set the action state to preempted
        		forwardServer_.setPreempted();
        		left.data = 1500;
        		right.data = 1500;
        		reached = false;
        		break;
      		}
      		if (error < maxband && error > minband)
      		{
      			reached = true;
        		left.data = 1500;
        		right.data = 1500;
        		ROS_INFO("%s thrusters stopped", ros::this_node::getName().c_str());
        		count++;
      		}

			feedback_.DistanceRemaining = error;
			forwardServer_.publishFeedback(feedback_);
      		
			pwm_left.publish(left);
   			pwm_right.publish(right);
      		ROS_INFO("%s pwm send to forward left %d", ros::this_node::getName().c_str(), left.data);
      		ROS_INFO("%s pwm send to forward right %d", ros::this_node::getName().c_str(), right.data);
      		ros::spinOnce();
			loop_rate.sleep();	
			previous_error_angle = error_angle;
		}

		if (reached)
		{
			result_.Result = reached;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			forwardServer_.setSucceeded(result_);
		}
	}

  	void setPID(float new_p, float new_i, float new_d)
  	{
    	kp = new_p;
    	ki = new_i;
    	kd = new_d;
	}
};
innerActionClass *object;
// dynamic reconfig

void yawCb(std_msgs::Float64 msg)
{
	previous_yaw_angle = yaw_angle;
	yaw_angle = msg.data;
	ROS_INFO("stabilizing");
	ROS_INFO("%f Yaw data", yaw_angle);
}
	
void callback(forward_motion::pidConfig &config, double level)
{
  ROS_INFO("%s ForwardServer: Reconfigure Request: p= %f i= %f d=%f error band=%f", ros::this_node::getName().c_str(),
           config.p, config.i, config.d);
  object->setPID(config.p, config.i, config.d);
}

void distanceCb(std_msgs::Float64 msg)
{
  // this is used to set the final angle after getting the value of first intial position
  //if (initData == false)
  //{
   // presentForwardPosition = msg.data;
   // previousForwardPosition = presentForwardPosition;
   // initData = true;
  //}
  //else
  //{
  if(msg.data)	
  {
    previousForwardPosition = presentForwardPosition;
    presentForwardPosition = msg.data;
  }
  else
  {
	previousForwardPosition = 0;
	presentForwardPosition = 0;
  }
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"forward");
	ros::NodeHandle n;
	double p_param, i_param, d_param;
	n.getParam("forward/kp", p_param);
	n.getParam("forward/ki", i_param);
	n.getParam("forward/kd", d_param);
	ros::Subscriber xdistance = n.subscribe<std_msgs::Float64>("forward_motion",1000,&distanceCb);
	ros:: Subscriber sub_ = n.subscribe<std_msgs::Float64>("yaw_angle",1000, &yawCb);
	ROS_INFO("%s Waiting for goal",ros::this_node::getName().c_str());
	object = new innerActionClass(ros::this_node::getName());

	  // register dynamic reconfig server.
	dynamic_reconfigure::Server<forward_motion::pidConfig> server;
	dynamic_reconfigure::Server<forward_motion::pidConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	// set launch file pid
	forward_motion::pidConfig config;
	config.p = p_param;
	config.i = i_param;
	config.d = d_param;
	callback(config, 0);
	ros::spin();
	return 0;
} 
