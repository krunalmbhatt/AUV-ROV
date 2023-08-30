#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <JHPWMPCA9685.h>
#include <ros/ros.h>
#include <message_files/pwm_msgs.h>

int front_pwm, left_pwm, back_pwm, top_pwm, right_pwm, bottom_pwm;
PCA9685 *pca9685 ;
int address = pca9685->getAddress();
int err = pca9685->openPCA9685();

void pwmCb(message_files::pwm_msgs msg)
{
    
	left_pwm = msg.left_pwm;
	right_pwm = msg.right_pwm;
	top_pwm = msg.top_pwm;
	bottom_pwm = msg.bottom_pwm;
	front_pwm = msg.front_pwm;
	back_pwm = msg.back_pwm;
	ROS_INFO("%d pwm left", left_pwm);
	ROS_INFO("%d pwm right", right_pwm);

	//if (err < 0)
	//	printf("Error: %d", pca9685->error);
    //else 
   // {
        //printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
       // pca9685->setAllPWM(0,0) ;
        //pca9685->reset() ;
        pca9685->setPWMFrequency(1500) ;
        // 27 is the ESC key
        //printf("Hit ESC key to exit\n");
        //if(pca9685->error >= 0 && getkey() != 27)
        //{
			pca9685->setPWM(0,0,1500) ;
			pca9685->setPWM(1,0,1500) ;
			pca9685->setPWM(2,0,1500) ;
			pca9685->setPWM(3,0,1500) ;
			pca9685->setPWM(4,0,1500) ;
			pca9685->setPWM(5,0,1500) ;
			sleep(1);
			pca9685->setPWM(1,0,1600);
			pca9685->setPWM(0,0,1600);
			pca9685->setPWM(2,0,1600) ;
			pca9685->setPWM(3,0,1600) ;
			pca9685->setPWM(4,0,1600) ;
			pca9685->setPWM(5,0,1600) ;
        //}
			sleep(1);
        	pca9685->setPWM(1,0,right_pwm);
			pca9685->setPWM(0,0,left_pwm);
			pca9685->setPWM(2,0,top_pwm) ;
			pca9685->setPWM(3,0,bottom_pwm) ;
			pca9685->setPWM(4,0,front_pwm) ;
			pca9685->setPWM(5,0,back_pwm) ;

    //}
 //   pca9685->closePCA9685();
}
int main(int argc, char **argv)
{
 
    ros::init(argc , argv , "pwm_generator");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("pwm" , 1000 , &pwmCb);
    ros::spin();
	return 0;
}
