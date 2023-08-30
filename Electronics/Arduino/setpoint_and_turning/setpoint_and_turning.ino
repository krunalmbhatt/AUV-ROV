#include <Wire.h>

#include <MS5837.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_BNO055.h>

#include <Servo.h>

float kpb=1,kib=2,kdb=3;
float pidb,pidb_p,pidb_i,pidb_d;
float kpp=1,kip=2,kdp=3;
float pidp,pidp_p,pidp_i,pidp_d;

Servo left , right, front1 , front2, back1 , back2 ;

MS5837 pressure;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float time,previous_time,elapsed_time;

float errorp,previous_errorp;
float errorb,previous_errorb;

float setpoint=0.75;
float depth;

float pwmdepth ,pwm;

float gyroX;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
bno.begin();
Wire.begin();
pressure.init();

left.attach(8);
right.attach(9);
front1.attach(10);
front2.attach(11);
back1.attach(12);
back2.attach(13);

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

time=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
previous_time=time;
time=millis();
elapsed_time=time-previous_time;

///////////PID DEPTH///////////
depth=pressure.depth();
errorp=depth-setpoint;

pidp_p=kpp*errorp;

if(0.01<errorp<0.05)
{
  pidp_i+=kip*errorp;
}

pidp_d=(errorp-previous_errorp)/elapsed_time;

pidp=pidp_p+pidp_i+pidp_d;

if(pidp>400)
{
  pidp=400;
}
if(pidp<-400)
{
  pidp=-400;
}

pwmdepth=1500+pidp;

if(pwmdepth>1900)
{
  pwmdepth=1900;
}
if(pwmdepth<1100)
{
  pwmdepth=1100;
}

left.writeMicroseconds(pwmdepth);
right.writeMicroseconds(pwmdepth);

front1.writeMicroseconds(1700);
front2.writeMicroseconds(1700);
back1.writeMicroseconds(1700);
back2.writeMicroseconds(1700);

///////////turning 90 degrees///////////
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
gyroX=euler.x();

errorb=90-gyroX;

pidb_p=kpb*errorb;

if(errorb<5)
{
pidb_i+=kib*errorb;
}

pidb_d=(errorb-previous_errorb)/elapsed_time;

pidb=pidb_p+pidb_i+pidb_d;

if(pidb>400)
{
  pidb=400;
}
if(pidb<-400)
{
  pidb=-400;
}

pwm=1500+pidb;

if(pwm>1900)
{
  pwm=1900;
}
if(pwm<1100)
{
  pwm=1100;
}


front1.writeMicroseconds(pwm);
front2.writeMicroseconds(pwm);
back1.writeMicroseconds(pwm);
back2.writeMicroseconds(pwm);

previous_errorb=errorb;
previous_errorp=errorp;



}
