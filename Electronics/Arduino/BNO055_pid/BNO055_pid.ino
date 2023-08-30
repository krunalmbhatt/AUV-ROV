#include <Adafruit_Sensor.h>

#include <Adafruit_BNO055.h>

#include <Wire.h>

#include <Servo.h>


Servo rollleft,rollright,pitchfront,pitchback,yawleft,yawright; // 6 motors for 3 direction controls

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float kiX=0,kpX=3,kdX=3; //pid constants
float kiY=0,kpY=3,kdY=3;
float kiZ=0,kpZ=3,kdZ=3;

float pidX,pid_pX=0,pid_iX=0,pid_dX=0; //total pid and other terms
float pidY,pid_pY=0,pid_iY=0,pid_dY=0;
float pidZ,pid_pZ=0,pid_iZ=0,pid_dZ=0;

float time,previous_time,elapsed_time; //time element

float errorX,previous_errorX; //errors
float errorY,previous_errorY;
float errorZ,previous_errorZ;
float desired_angle=0;

float headingY,headingZ;//for correcting the zero position
float gyroX,gyroY,gyroZ;//angle read by gyrometer

int throttleX=1500;
int throttleY=1500;
int throttleZ=1500;

float Rpwmright,Rpwmleft; // pwm to be given to different esc 
float Ypwmright,Ypwmleft;
float Ppwmfront,Ppwmback;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
bno.begin();

rollleft.attach(8);
rollright.attach(9);
pitchfront.attach(10);
pitchback.attach(11);
yawleft.attach(12);
yawright.attach(13);

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
headingY=euler.y();
headingZ=euler.z();
time=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
 previous_time=time;
 time=millis();
 elapsed_time=time-previous_time;

 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

gyroX=euler.x();
gyroY=euler.y();
gyroZ=euler.z();

if(gyroX>180)
{
  gyroX=gyroX-360;
}
gyroY=gyroY-headingY;
gyroZ=gyroZ-headingZ;

////////////PID X=YAW///////////

errorX=gyroX-desired_angle;

pid_pX=kpX*errorX;

if(-3<errorX<3)
{
  pid_iX+=kiX*errorX;
}

pid_dX=kdX*((errorX-previous_errorX)/elapsed_time);

pidX=pid_pX+pid_iX+pid_dX;

if(pidX>400)
{
  pidX=400;
}
if(pidX<-400)
{
  pidX=-400;
}

Ypwmright=throttleX+pidX;
Ypwmleft=throttleX-pidX;

if(Ypwmright>1900)
{
  Ypwmright=1900;
}
if(Ypwmright<1100)
{
  Ypwmright=1100;
}
if(Ypwmleft>1900)
{
  Ypwmleft=1900;
}
if(Ypwmleft<1100)
{
  Ypwmleft=1100;
}

/*Serial.print("Ypwmright=");
Serial.print(Ypwmright);
Serial.print(" Ypwmleft=");
Serial.print(Ypwmleft); 


Serial.print(" angle X=");
Serial.println(errorX);*/

/*yawleft.writeMicroseconds(Ypwmleft);
yawright.writeMicroseconds(Ypwmright);*/

previous_errorX=errorX;

////////////PID Y=ROLL///////////

errorY=gyroY-desired_angle;

pid_pY=kpY*errorY;

if(-3<errorY<3)
{
  pid_iY+=kiY*errorY;
}

pid_dY=kdY*((errorY-previous_errorY)/elapsed_time);

pidY=pid_pY+pid_iY+pid_dY;

if(pidY>400)
{
  pidY=400;
}
if(pidY<-400)
{
  pidY=-400;
}

Rpwmright=throttleY-pidY;
Rpwmleft=throttleY+pidY;

if(Rpwmright>1900)
{
  Rpwmright=1900;
}
if(Rpwmright<1100)
{
  Rpwmright=1100;
}
if(Rpwmleft>1900)
{
  Rpwmleft=1900;
}
if(Rpwmleft<1100)
{
  Rpwmleft=1100;
}
/*Serial.print("Rpwmright=");
Serial.print(Rpwmright);
Serial.print(" Rpwmleft=");
Serial.print(Rpwmleft);

Serial.print(" angle Y=");
Serial.println(errorY);*/

/*rollleft.writeMicroseconds(Rpwmleft);
rollright.writeMicroseconds(Rpwmright);*/

previous_errorY=errorY;

////////////PID Z=PITCH///////////

errorZ=gyroZ-desired_angle;

pid_pZ=kpZ*errorZ;

if(-3<errorZ<3)
{
  pid_iZ+=kiZ*errorZ;
}

pid_dZ=kdZ*((errorZ-previous_errorZ)/elapsed_time);

pidZ=pid_pZ+pid_iZ+pid_dZ;

if(pidZ>400)
{
  pidZ=400;
}
if(pidZ<-400)
{
  pidZ=-400;
}

Ppwmback=throttleZ+pidZ;
Ppwmfront=throttleZ-pidZ;

if(Ppwmfront>1550)
{
  Ppwmfront=1550;
}
if(Ppwmfront<1450)
{
  Ppwmfront=1450;
}
if(Ppwmback>1550)
{
  Ppwmback=1550;
}
if(Ppwmback<1450)
{
  Ppwmback=1450;
}

Serial.print(" Ppwmfront=");
Serial.print(Ppwmfront,0);
Serial.print(" Ppwmback=");
Serial.print( Ppwmback,0);


Serial.print(" angle z=");
Serial.println(errorZ);

/*pitchback.writeMicroseconds(Ppwmback);
pitchfront.writeMicroseconds(Ppwmfront);*/

previous_errorZ=errorZ;

delay(100);
}
