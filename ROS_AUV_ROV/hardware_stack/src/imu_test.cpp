#include <ros/ros.h>
#include "message_files/imu_msgs.h"

void callback(const message_files::imu_msgs::ConstPtr& msg)
{
    ROS_INFO("getting data");
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f",msg->roll, msg->pitch, msg->yaw);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_test");
	ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<message_files::imu_msgs>("imu_data", 1000, callback);
    ROS_INFO("getting data");
    ros::spin();
	return 0;
}
