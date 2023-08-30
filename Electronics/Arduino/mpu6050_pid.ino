#include <Servo.h>

#include <Wire.h>

#include <MPU6050_tockn.h>


MPU6050 mpu6050(Wire);
Servo motor1;
Servo motor2;

float pidX,pid_pX=0,pid_iX=0,pid_dX=0;
float pidY,pid_pY=0,pid_iY=0,pid_dY=0;

float kiX=1,kdX=2,kpX=3;
float kiY=1,kdY=2,kpY=3;

float desired_angle=0,gyroX=0,gyroY=0;

float errorX,previous_errorX=0,elapsed_time=0,time,previous_time,errorY,previous_errorY=0;

float pwmrightX,pwmleftX;
float pwmrightY,pwmleftY;

int throttle=1500;

float headingX,headingY;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.begin();
mpu6050.begin();
mpu6050.calcGyroOffsets(true);
motor1.attach(9);
motor2.attach(8);
time=millis();
headingX=mpu6050.getAngleX();
headingY=mpu6050.getAngleY();
}

void loop() {
  // put your main code here, to run repeatedly:
  previous_time=time;
time=millis();
elapsed_time=(time-previous_time)/1000;

mpu6050.update();

gyroX=mpu6050.getAngleX();
gyroY=mpu6050.getAngleY();

//////////////PID X/////////////
if(gyroX>180)
{
  gyroX=gyroX-360;
}

gyroX=headingX-gyroX;

errorX=desired_angle-gyroX;

pid_pX=kpX*errorX;

pid_dX=kdX*((errorX-previous_errorX)/elapsed_time);

if(-3<errorX<3)
{
  pid_iX+=errorX*kiX;
}

pidX=pid_pX+pid_iX+pid_dX;

if(pidX<-400)
{
  pidX=-400;
}
if(pidX>400)
{
  pidX=400;
}

pwmrightX=throttle+pidX;
pwmleftX=throttle-pidX;

Serial.print("gyroX angle=");
Serial.println(gyroX);

/*Serial.print("pwmrightX=");
Serial.print(pwmrightX);

Serial.print("pwmleftX=");
Serial.println(pwmleftX);*/

/*motor1.writeMicroseconds(pwmrightX);
motor2.writeMicroseconds(pwmleftX);*/

previous_errorX=errorX;

//////////////PID Y/////////////

if(gyroY>180)
{
  gyroY=gyroY-360;
}

gyroY=headingY-gyroY;

errorY=desired_angle-gyroY;

pid_pY=kpY*errorY;

pid_dY=kdY*((errorY-previous_errorY)/elapsed_time);

if(-3<errorY<3)
{
  pid_iY+=errorY*kiY;
}

pidY=pid_pY+pid_iY+pid_dY;

if(pidY<-400)
{
  pidY=-400;
}
if(pidY>400)
{
  pidY=400;
}

pwmrightY=throttle+pidY;
pwmleftY=throttle-pidY;

Serial.print("gyroY angle=");
Serial.print(gyroY);

/*Serial.print("pwmrightY=");
Serial.print(pwmrightY);

Serial.print("pwmleftY=");
Serial.println(pwmleftY);*/

/*motor1.writeMicroseconds(pwmrightY);
motor2.writeMicroseconds(pwmleftY);*/

previous_errorY=errorY;
delay(100);
}
