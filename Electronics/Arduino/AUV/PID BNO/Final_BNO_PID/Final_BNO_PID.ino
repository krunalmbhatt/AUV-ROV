#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"

Servo rollLeft,rollRight,pitchFront,pitchBack,yawLeft,yawRight; 
MS5837 sensor;

Adafruit_BNO055= Adafruit_BNO055(55,0x28);

float kX=0,kpX=6,kdX=6; //pid constants
float kY=0,kpY=6,kdY=6;
float kZ=0,kpZ=6,kdZ=6;
float kD=0,kpD=6,kdD=6;

float pidX,pid_pX=0,pid_iX=0,pid_dX=0; //final pid and other terms and values
float pidY,pid_pY=0,pid_iY=0,pid_dY=0;
float pidZ,pid_pZ=0,pid_iZ=0,pid_dZ=0;
float pidD,pid_pD=0,pid_iD=0,pid_dD=0;

float time,previous_time,elapsed_time;

float errorX,previous_errorX; //errors
float errorY,previous_errorY;
float errorZ,previous_errorZ;
float errorD,previous_errorD;
float desired_angle=0;

float headingY,headingZ;//for correcting the position
float gyroX,gyroY,gyroZ;//angle read by gyrometer

int throttleX=1500;
int throttleY=1500;
int throttleZ=1500;
int throttleD=1500;             // change later at execution time

float Rpwmright,Rpwmleft; // pwm to be given to different esc 
float Ypwmright,Ypwmleft;
float Ppwmfront,Ppwmback;
float Dpwmup, Dpwmdown;

float desiredDepth,originalDepth,errorDepth;


void setup()
{
  // put your setup code here, to run once:
Serial.begin(9600);
bno.begin();

rollLeft.attach(8);
rollRight.attach(9);
pitchFront.attach(10);
pitchBack.attach(11);
yawLeft.attach(12);
yawRight.attach(13);

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
headingY=euler.y();
headingZ=euler.z();
time=millis();

Serial.println("Starting pressure sensor");
  Wire.begin();
while (!sensor.init()) 
  {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  desiredDepth = sensor.depth();
  Serial.print("Depth: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" m");
}







void loop()
{
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
//gyroY=gyroY-headingY;
//gyroZ=gyroZ-headingZ;

                                                                    ////////////PID X=YAW///////////

errorX=desired_angle-gyroX; //taking the value of error from gyro as desired angle is 0

pid_pX=kpX*errorX;   // error * proportional constant

if(-3<errorX<3)     // jo error bov nani hoi to 
{
  pid_iX+=kX*errorX;    // pid integral X = integral error of X* error X (je gyro ma thi madi che)
}

pid_dX=kdX*((errorX-previous_errorX)/elapsed_time);  //otherwise value overshoot kari gai hase to ene stabilize krva mate error difference /time 

pidX=pid_pX+pid_iX+pid_dX; //total pid for yaw

if(pidX>400)                // jo difference (1900-1500) je pidX che e 400 thi vadhi jai to ene 400 kari ape where 1500 is steady state and 1900 is full throttle
{
  pidX=400;
}
if(pidX<-400)             // jo difference -400 etle 1500 thi ochu hoi to reverse java mande ane reverse ma bhi limit is 1100 so if ena thi ochu jai to 1100 set kari ape
{
  pidX=-400;
}

Ypwmright=throttleX-pidX;   // total pid value ne 1500 ma add or subtract krse to maintain setpoint
Ypwmleft=throttleX+pidX;

if(Ypwmright>1900)         // jo ema bhi pwm ni value 1900 thi vadhare to set it to 1900
{
  Ypwmright=1900;
}
if(Ypwmright<1100)       // successive limits
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

previous_errorX=errorX;  /// first time khali setpoint value aavse then next time thi error avse if nescessary

                                                                   ////////////PID Y=ROLL///////////

errorY=desired_angle-gyroY;

pid_pY=kpY*errorY;

if(-3<errorY<3)
{
  pid_iY+=kY*errorY;
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

previous_errorY=errorY;

                                                                              ////////////PID Z=PITCH///////////

errorZ=desired_angle-gyroZ;

pid_pZ=kpZ*errorZ;

if(-3<errorZ<3)
{
  pid_iZ+=kZ*errorZ;
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

if(Ppwmfront>1900)
{
  Ppwmfront=1900;
}
if(Ppwmfront<1100)
{
  Ppwmfront=1100;
}
if(Ppwmback>1900)
{
  Ppwmback=1900;
}
if(Ppwmback<1100)
{
  Ppwmback=1100;
}
Serial.print(" Ypwmright=");
Serial.print(Ypwmright,0);
Serial.print(" Ypwmleft=");
Serial.println( Ypwmleft,0);

Serial.print(" Rpwmright=");
Serial.print(Rpwmright,0);
Serial.print(" Rpwmleft=");
Serial.println( Rpwmleft,0);

Serial.print(" Ppwmfront=");
Serial.print(Ppwmfront,0);
Serial.print(" Ppwmback=");
Serial.println( Ppwmback,0);

Serial.print(" angle x=");
Serial.print(gyroX);
Serial.print(" angle Y=");
Serial.print(gyroY);
Serial.print(" angle z=");
Serial.println(gyroZ);

previous_errorZ=errorZ;

                                                                          //////// PID - Depth /////////

sensor.read();
Serial.print("Depth: "); 
Serial.print(sensor.depth()); 
Serial.println(" m");

errorDepth=desiredDepth-originalDepth;

pid_pD=kpD*errorD;

if(-3<errorD<3)
{
  pid_iD+=kD*errorD;
}

pid_dD=kdD*((errorD-previous_errorD)/elapsed_time);

pidD=pid_pD+pid_iD+pid_dD;

if(pidD>400)
{
  pidD=400;
}
if(pidD<-400)
{
  pidD=-400;
}

Dpwmup=throttleD+pidZ;
Dpwmdown=throttleD-pidZ;

if(Dpwmup>1900)
{
  Dpwmup=1900;
}
if(Dpwmup<1100)
{
  Dpwmup=1100;
}
if(Dpwmdown>1900)
{
  Dpwmdown=1900;
}
if(Dpwmdown<1100)
{
  Dpwmdown=1100;
}

delay(1000);
}


/*----------------------------------------------------------------------*/



/*#include <Servo.h>

byte servoPin = 9;
Servo servo;

void setup() {
  
 Serial.begin(9600);
 servo.attach(servoPin);

 servo.writeMicroseconds(1500); // send "stop" signal to ESC.

 delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  
  Serial.println("Enter PWM signal value 1100 to 1900, 1500 to stop");
  
  while (Serial.available() == 0);
  
  int val = Serial.parseInt(); 
  
  if(val < 1100 || val > 1900)
  {
    Serial.println("not valid");
  }
  else
  {
    servo.writeMicroseconds(val); // Send signal to ESC.
  }
}

--------------------------------------------------------------------------------------------------------------------



#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

void setup() 
{
  
  Serial.begin(9600);
  
  Serial.println("Starting");
  
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  sensor.read();
  Serial.print("Depth: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" m");
  
  delay(1000);
}*/
