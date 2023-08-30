#include <ros/ros.h>
#include "message_files/displacement.h"

void cb(message_files::displacement msg)
{
	ROS_INFO("distance X:%f, Y:%f, Z:%f", msg.x, msg.y, msg.z);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "distance_test");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<message_files::displacement>("linear_displacement", 1000, &cb);
	ros::spin();
	return 0;
}
