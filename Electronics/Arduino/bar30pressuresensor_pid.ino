#include <Servo.h>

#include <Wire.h>

#include <MS5837.h>

Servo left ;
Servo right;

MS5837 sensor;

float pid,pid_p=0,pid_i=0,pid_d=0;
float kp=3,ki=2,kd=1;

float time,elapsed_time,previous_time;

float error,previous_error;
float depth;
float desired_depth=0.5;

float pwm;

int throttle=1500;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.begin();
sensor.init();
left.attach(8);
right.attach(9);
time=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  previous_time=time;
  time=millis();
  elapsed_time=time-previous_time;

  depth=sensor.depth();
  error= depth-desired_depth;

  //print("depth=");
  //print(depth);

  pid_p+=kp*error;

  if(0.01<error<0.02)
  {
    pid_i+=ki*error;
  }

  pid_d+=kd*(error-previous_error)/elapsed_time;

  pid=pid_p+pid_i+pid_d;

  if(pid>400)
  {
    pid=400;
  }
  if(pid<-400)
  {
    pid=-400;
  }

  pwm=throttle+pid;

  if(pwm>1900)
  {
    pwm=1900;
  }
  if(pwm<1100)
  {
    pwm=1100;
  }

  left.writeMicroseconds(pwm);
  right.writeMicroseconds(pwm);

  }
