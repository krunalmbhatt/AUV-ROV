#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>
 
Servo pitchServo;
Servo rollServo;
Servo yawServo;


float k1= 0.5;   //Proportionality     
float k2=70;     // derivative
float k3=0.001;   // integral

int milliold;
int millinew;
int dt;

float rollTarget=0;
float rollActual;
float rollErrorOld;
float rollError=0;
float rollErrorChange;
float rollErrorSlope=0;    // change on error by time
float rollErrorArea=0;
float rollServoVal=90;

float pitchTarget=0;
float pitchActual;
float pitchError=0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope=0;
float pitchErrorArea=0;
float pitchServoVal=90;

float yawTarget=0;
float yawActual;
float yawError=0;
float yawErrorOld;
float yawErrorChange;
float yawErrorSlope=0;
float yawErrorArea=0;
float yawServoVal=90;

float headingY,headingZ;
float gyroX,gyroY,gyroZ;//angle read by gyrometer

int throttleX=1500;
int throttleY=1500;
int throttleZ=1500;

float Rpwmright,Rpwmleft; // pwm to be given to different esc 
float Ypwmright,Ypwmleft;
float Ppwmfront,Ppwmback;

#define BNO055_SAMPLERATE_DELAY_MS (100)
 
Adafruit_BNO055 myIMU = Adafruit_BNO055();
 
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
myIMU.begin();

bno.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
rollServo.attach(2);
pitchServo.attach(3);
yawServo.attach(4);
 
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
headingY=euler.y();
headingZ=euler.z();
time=millis();
}
 
void loop() 
{
  // put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mg = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mg);

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

gyroX=euler.x();
gyroY=euler.y();
gyroZ=euler.z();

 
rollActual=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));    ///////////doubt
pitchActual=asin(2*(q0*q2-q3*q1));
 
rollActual=rollActual/(2*3.141592654)*360;
pitchActual=pitchActual/(2*3.141592654)*360;
yawActual = yawActual/(2*3.141592654)*360;

 
milliold=millinew;
millinew=millis();
dt= millinew-milliold;

rollErrorOld=rollError;
rollError=rollTarget-rollActual;
rollErrorChange = rollError-rollErrorOld;
rollErrorSlope= rollErrorChange/dt;
rollErrorArea=rollErrorArea+rollError*dt;

pitchErrorOld=pitchError;
pitchError=pitchTarget-pitchActual;
pitchErrorChange = pitchError-pitchErrorOld;
pitchErrorSlope= pitchErrorChange/dt;
pitchErrorArea=pitchErrorArea+pitchError*dt;

yawErrorOld=yawError;
yawError=yawTarget-yawActual;
yawErrorChange = yawError-yawErrorOld;
yawErrorSlope= yawErrorChange/dt;
yawErrorArea=yawErrorArea+yawError*dt;

rollServoVal = rollServoVal+k1*rollError+k2*rollErrorSlope+k3*rollErrorArea;
rollServo.write(rollServoVal);

pitchServoVal = pitchServoVal+k1*pitchError+k2*pitchErrorSlope+k3*pitchErrorArea;
pitchServo.write(pitchServoVal);

yawServoVal = yawServoVal+k1*yawError+k2*yawErrorSlope+k3*yawErrorArea;
yawServo.write(yawServoVal);

Serial.println(rollTarget);
Serial.print(",");
Serial.print(rollActual);
Serial.print(",");
Serial.println(pitchTarget);
Serial.print(",");
Serial.print(pitchActual);
Serial.print(",");
Serial.println(yawTarget);
Serial.print(",");
Serial.print(yawActual);
Serial.print(",");

/*Serial.print(accel);
Serial.print(",");
Serial.print(gyro);
Serial.print(",");
Serial.print(mg);
Serial.print(",");
Serial.println(system);
*/ 
delay(BNO055_SAMPLERATE_DELAY_MS);
}
