#include <ros/ros.h>
#include <message_files/pwm_msgs.h>
#include <std_msgs/UInt16.h>

int front_pwm, left_pwm, back_pwm, top_pwm, right_pwm, bottom_pwm;

void pwmCb(message_files::pwm_msgs msg)
{
    ros::NodeHandle nh;
    ros::Publisher pub_left = nh.advertise<std_msgs::UInt16>("left_pwm",1000);
    ros::Publisher pub_right = nh.advertise<std_msgs::UInt16>("right_pwm",1000);
    ros::Publisher pub_top = nh.advertise<std_msgs::UInt16>("top_pwm",1000);
    ros::Publisher pub_bottom = nh.advertise<std_msgs::UInt16>("bottom_pwm",1000);
    ros::Publisher pub_front = nh.advertise<std_msgs::UInt16>("front_pwm",1000);
    ros::Publisher pub_back = nh.advertise<std_msgs::UInt16>("back_pwm",1000);
    std_msgs::UInt16 left_pwm_data, right_pwm_data, top_pwm_data, bottom_pwm_data, front_pwm_data, back_pwm_data;
   	left_pwm_data.data = msg.left_pwm;
	right_pwm_data.data = msg.right_pwm;
	top_pwm_data.data = msg.top_pwm;
	bottom_pwm_data.data = msg.bottom_pwm;
	front_pwm_data.data = msg.front_pwm;
	back_pwm_data.data = msg.back_pwm;
	pub_left.publish(left_pwm_data);
	ROS_INFO("Left PWM Published %d" , left_pwm_data.data);
	pub_right.publish(right_pwm_data);
	ROS_INFO("Right PWM Published %d" , right_pwm_data.data);
	pub_top.publish(top_pwm_data);
	ROS_INFO("Top PWM Published %d" , top_pwm_data.data);
	pub_bottom.publish(bottom_pwm_data);
	ROS_INFO("Bottom PWM Published %d" , bottom_pwm_data.data);
	pub_front.publish(front_pwm_data);
	ROS_INFO("Front PWM Published %d" , front_pwm_data.data);
	pub_back.publish(back_pwm_data);
	ROS_INFO("Back PWM Published %d" , back_pwm_data.data);
}
int main(int argc, char **argv)
{
 
    ros::init(argc , argv , "arduino");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("pwm" , 1000 , &pwmCb);
    ros::spin();
	return 0;
}
