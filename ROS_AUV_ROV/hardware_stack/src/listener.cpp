#include "ros/ros.h"
#include "message_files/imu_msgs.h"

void chatterCallback(message_files::imu_msgs msg)
{
	 ROS_INFO("I heard: [%f]", msg.data);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 1000, &chatterCallback);
	ros::spin();
	return 0;
}