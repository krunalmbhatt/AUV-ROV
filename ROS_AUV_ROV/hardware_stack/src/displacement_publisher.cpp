#include <ros/ros.h>
#include "message_files/displacement.h"
#include <RTIMULib.h>
static const float G_TO_MPSS = 9.80665;

    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);
    float v_x, v_y, v_z;
float *velocity_x , *velocity_y, *velocity_z, *distance_x, *distance_y, *distance_z;
float vc_x()
{
    for(int i = 0; i < 1000; i++)
    {
    	if (imu->IMURead())
    	{
    		RTIMU_DATA imu_data = imu->getIMUData();
    		*velocity_x += (imu_data.accel.x() * G_TO_MPSS)*10.00; 
    	}	
    }
    return *velocity_x;
}
float vc_y()
{
    for(int i = 0; i < 1000; i++)
    {
    	if (imu->IMURead())
    	{
    		RTIMU_DATA imu_data = imu->getIMUData();
    		*velocity_y += (imu_data.accel.y() * G_TO_MPSS)*10.00;
    	}	
    }
    return *velocity_y;
}
float vc_z()
{
    for(int i=0; i < 1000; i++)
    {
    	if (imu->IMURead())
    	{
    		RTIMU_DATA imu_data = imu->getIMUData();
    		*velocity_z += (imu_data.accel.z() * G_TO_MPSS)*10.00;
    	}	
    }
    return *velocity_z;
}
float dc_x()
{
    for(int i = 0; i < 1000 ; i++)
    {
    	if(imu->IMURead())
    	{
    		v_x=vc_x();
    		*distance_x += v_x * 10.00;    		
    	}
    }
	return *distance_x;
}
float dc_y()
{
	for(int i = 0; i < 1000 ; i++)
    {
        if(imu->IMURead())
    	{
    		v_y=vc_y();
    		*distance_y += v_y * 10.00;
    	}
    }
	return *distance_y;
}
float dc_z()
{
    for(int i = 0; i < 1000; i++)
    {
    	if(imu->IMURead())
    	{
    		v_z=vc_z();
    		*distance_z += v_z * 10.00;
    	}
    }
	return *distance_z;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "displacement_publisher");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<message_files::displacement>("linear_displacement",1000);

    message_files::displacement dc;


    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    imu->IMUInit();

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    while(ros::ok())
    {
    	dc.x = dc_x();
    	dc.y = dc_y();
    	dc.z = dc_z();
    	ROS_INFO("distance traveled in X:%f , Y:%f , Z:%f", dc.x , dc.y , dc.z);
    	ros::spin();
    	pub.publish(dc);
    }

	return 0;
}