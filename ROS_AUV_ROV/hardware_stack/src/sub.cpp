  #include "ros/ros.h"
  #include "message_files/imu_msgs.h"

   void chatterCallback(const message_files::imu_msgs::ConstPtr& msg)
   {
     ROS_INFO("I heard: [%f]", msg->data);
   }
   
   int main(int argc, char **argv)
   {

     ros::init(argc, argv, "sub");

     ros::NodeHandle n;
   
   ros::Subscriber sub = n.subscribe<message_files::imu_msgs>("chatter", 1000, chatterCallback);

   ros::spin();
 
   return 0;
}
