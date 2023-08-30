#include <ros/ros.h>
#include <std_msgs/UInt16.h>


void RightpwmCb(std_msgs::UInt16 msg)
{
	ROS_INFO("Right PWM Published %d" , msg.data);
}

void LeftpwmCb(std_msgs::UInt16 msg)
{
	ROS_INFO("Left PWM Published %d" , msg.data);
}

int main(int argc, char **argv)
{
 
    ros::init(argc , argv , "arduino");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("right_pwm" , 1000 , &RightpwmCb);
    ros::Subscriber sub1 = n.subscribe("left_pwm" , 1000 , &LeftpwmCb);
    ros::spin();
	return 0;
}
