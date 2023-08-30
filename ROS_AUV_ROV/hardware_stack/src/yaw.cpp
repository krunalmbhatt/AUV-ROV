#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <message_files/imu_msgs.h>
#include <std_msgs/String.h>

void yawCb(std_msgs::Float64 msg)
{
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<message_files::imu_msgs>("imu_data",1000);
	ROS_INFO("subscribing data");
	message_files::imu_msgs value;
	value.yaw = msg.data;
	ROS_INFO("yaw value: %f ", msg.data);
	pub.publish(value);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "yaw");
	ros::NodeHandle n;
		ROS_INFO("waiting for data");
		ros::Subscriber sub = n.subscribe("depth", 1000, &yawCb);
		ROS_INFO("data incoming");
		ros::spin();
	ROS_INFO("in main");
}
