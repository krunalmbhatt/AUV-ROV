#include "ros/ros.h"
  #include "message_files/imu_msgs.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pub");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<message_files::imu_msgs>("chatter", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	while (ros::ok())
	{
		message_files::imu_msgs msg;
		msg.data = count;
		ROS_INFO("%f", msg.data);
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
