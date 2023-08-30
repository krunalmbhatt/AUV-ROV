#include <Servo.h>

#include <MagneticSensorLsm303.h>


MagneticSensorLsm303 compass;
Servo left;
Servo right;

int i=0;
int throttle=1500;

float time,previous_time,elapsed_time;
float previous_error,error;

float pid,pid_p=0,pid_i=0,pid_d=0;
float kp=1,ki=0,kd=0;

float pwmright,pwmleft;

void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 compass.init();
 compass.enable();
 time=millis();
 left.attach(8);
 right.attach(9);
 }

void loop() {
  // put your main code here, to run repeatedly:
  previous_time=time;
  time=millis();
  elapsed_time=time-previous_time;
  
  compass.read();
  float heading2,heading1;
  if(i<2)
  {
    heading1= compass.getNavigationAngle();
    i++;
    delay(500);
  }
  heading2=compass.getNavigationAngle();
  
   error = heading2-heading1;

  Serial.print("error =");
  Serial.print(error);

     //////////PID/////////
pid_p=kp*error;

if(-3<error<3)
{
  pid_i+=error*ki;
}

pid_d=kd*((error-previous_error)/elapsed_time);

pid=pid_p+pid_i+pid_d;

if(pid>400)
{
  pid=400;
}
if(pid<-400)
{
  pid=-400;
}

pwmright=throttle+pid;
pwmleft=throttle-pid;

if(pwmright>1900)
{
  pwmright=1900;
}
if(pwmright<1100)
{
  pwmright=1100;
}

if(pwmleft>1900)
{
  pwmleft=1900;
}
if(pwmleft<1100)
{
  pwmleft=1100;
}

/*left.writeMicroseconds(pwmleft);
right.writeMicroseconds(pwmright);*/

Serial.print("pwmright=");
Serial.print(pwmright);

Serial.print("pwmleft=");
Serial.println(pwmleft);

previous_error=error;

  delay(10);
}
