#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"
MS5837 sensor;
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
int incomingByte;
int reset_yaw;
Servo thruster1;
Servo thruster4;
Servo thruster2;
Servo thruster3;

Servo thruster5;
Servo thruster8;
Servo thruster6;
Servo thruster7;

float set_depth=0;
float current_depth=0;
float depth_error=0;
float depth_p=110;
float depth_i=0;
float depth_d=10;


/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/
 
//For Sensor BNO055 from here
 #define BNO055_SAMPLERATE_DELAY_MS (0)
 Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int constantdiffyaw,constantdiffpitch,constantdiffroll;
//Till here






float finalpwm5, finalpwm6, finalpwm7, finalpwm8;



float elapsedTimey, timey, timePrevy;
float elapsedTimep, timep, timePrevp;
float elapsedTimer, timer, timePrevr;
int fd=0;
int lr=0;
float PIDy, pwmLefty, pwmRighty, errory, previous_errory;
float PIDp, pwmfrontp, pwmbackp, errorp, previous_errorp;
float PIDr, pwmLeftr, pwmRightr, errorr, previous_errorr;

float pid_py=0;
float pid_iy=0;
float pid_dy=0;

float pid_pp=0;
float pid_ip=0;
float pid_dp=0;

float pid_pr=0;
float pid_ir=0;
float pid_dr=0;

/////////////////PID CONS/TANTS FOR YAW/////////////////
double kpy=5;//3.55
double kiy=0.001;//0.003
double kdy=5;//2.05
//////////////////////////////////////////////////////

/////////////////PID CONSTANTS FOR PITCH/////////////////
double kpp=-6;//3.55
double kip=-0.003;//0.003
double kdp=-2;//2.05
///////////////////////////////////////////////

/////////////////PID CONSTANTS FOR ROLL/////////////////
double kpr=5;//3.55
double kir=0.003;//0.003
double kdr=2;//2.05
///////////////////////////////////////////////


double throttle=1500; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  Wire.begin(); //begin the wire comunication
    Serial.println("Starting");

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
 
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
 
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  //For yaw
  thruster1.attach(5);//attatch the right motor to pin 3
  thruster2.attach(10);//attatch the left motor to pin 4
  thruster4.attach(5);  //attatch the left motor to pin 5
  thruster3.attach(7);//attatch the left motor to pin 6

 thruster5.attach(4);//attatch the left motor to pin 7
  thruster8.attach(3);//attatch the left motor to pin 8
   thruster6.attach(11);//attatch the left motor to pin 9
    thruster7.attach(12);//attatch the left motor to pin 10
 

  timey = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
   
  thruster1.writeMicroseconds(1500);  
  thruster2.writeMicroseconds(1500);
   thruster4.writeMicroseconds(1500);
    thruster3.writeMicroseconds(1500);

     thruster5.writeMicroseconds(1500);
     thruster8.writeMicroseconds(1500);
     thruster6.writeMicroseconds(1500);
     thruster7.writeMicroseconds(1500);

   
  delay(2000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/

//thruster8.writeMicroseconds(1700);
//delay(2000);            
  //Setup for BNO055 Sensor from here
 Serial.begin(115200);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);


   //apna code
    sensors_event_t event;
   bno.getEvent(&event);

 
  int yaw= event.orientation.x;
  reset_yaw=event.orientation.x;
  constantdiffyaw=yaw;
      int pitch= event.orientation.y;
  constantdiffpitch=pitch;
    int roll= event.orientation.z;
  constantdiffroll=roll;

  //Till here
                 
}//end of setup void


void loop() {
 
  if(Serial.available()>0){
  if (Serial.read()=='w'){
    fd=200;
    lr=0;}
   if (Serial.read()=='s'){
    fd=-200;
    lr=0;}
   if (Serial.read()=='a'){
    fd=0;
    lr=200;}
   if (Serial.read()=='d'){
    fd=0;
    lr=-200;}
  if (Serial.read()=='q'){
    constantdiffyaw=constantdiffyaw-15;
    Serial.println(constantdiffyaw);}
  if (Serial.read()=='e'){
    constantdiffyaw=constantdiffyaw+15;
    Serial.println(constantdiffyaw);}
  if (Serial.read()=='r'){
    constantdiffyaw=reset_yaw;}
    if (Serial.read()=='i'){
    set_depth=set_depth-0.1;}
     if (Serial.read()=='k'){
    set_depth=set_depth+0.1;}    
    //  else if (Serial.read()=='y'){
    //fd=0;
   // lr=0;}
    }
  sensor.read();
current_depth=sensor.depth();
depth_error=current_depth-set_depth;


int depth_p_pwm=depth_p*depth_error;

Serial.println(' ');
Serial.print("Current_Depth");
Serial.println(current_depth);


/////////////////////////////I M U/////////////////////////////////////
    timePrevy = timey;  // the previous time is stored before the actual time read
    timey = millis();  // actual time read
    elapsedTimey = (timey - timePrevy) / 1000;
   
   
//Loop for BNO055 Sensor  from here
    sensors_event_t event;
  bno.getEvent(&event);
 
  int yaw= event.orientation.x-constantdiffyaw;
    if(yaw<0)
   {
    yaw=yaw+360;
    }
//written below if statement will show 180 to 360 degree as 0 to -180 degree.
    if(yaw>180)
  {
    yaw=yaw-360;
    }



 int pitch= event.orientation.y-constantdiffpitch;
    if(pitch<0)
   {
    pitch=pitch+360;
    }
//written below if statement will show 180 to 360 degree as 0 to -180 degree.
    if(pitch>180)
  {
    pitch=pitch-360;
    }

   
  int roll= event.orientation.z-constantdiffroll;
 if(roll<0)
   {
    roll=roll+360;
    }
//written below if statement will show 180 to 360 degree as 0 to -180 degree.
    if(roll>180)
  {
    roll=roll-360;
    }
  //Till here

 
/*///////////////////////////P I D for yaw///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and
*the real measured angle*/
errory = yaw - desired_angle;
   
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_py = kpy*errory;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <errory <3)
{
  pid_iy = pid_iy+(kiy*errory);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time.
Finnaly we multiply the result by the derivate constant*/

pid_dy = kdy*((errory - previous_errory)/elapsedTimey);

/*The final PID values is the sum of each of this 3 parts*/
PIDy = pid_py + pid_iy + pid_dy;

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PIDy < -400)
{
  PIDy=-400;
}
if(PIDy > 400)
{
  PIDy=400;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLefty = throttle - PIDy;
pwmRighty = throttle + PIDy;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRighty < 1100)
{
  pwmRighty= 1100;
}
if(pwmRighty > 1900)
{
  pwmRighty=1900;
}
//Left
if(pwmLefty < 1100)
{
  pwmLefty= 1100;
}
if(pwmLefty > 1900)
{
  pwmLefty=1900;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
 Serial.print("yaw:");
  Serial.print(yaw);
  Serial.print("  ");

 //left side ke thrusters
 int pwmm1=pwmLefty+fd-lr;
 int pwmm2=pwmRighty+fd+lr;
 int pwmm3=pwmRighty+fd-lr;
 int pwmm4=pwmLefty+fd+lr;
 
thruster1.writeMicroseconds(pwmLefty+fd-lr);
thruster4.writeMicroseconds(pwmLefty+fd+lr);

//right side ke thrusters
thruster2.writeMicroseconds(pwmRighty+fd+lr);
thruster3.writeMicroseconds(pwmRighty+fd-lr);




previous_errory = errory; //Remember to store the previous error.
Serial.print("t1=");
Serial.print(pwmm1);
Serial.print(" ");
Serial.print("t2=");
Serial.print(pwmm2);
Serial.print(" ");
Serial.print("t3=");
Serial.print(pwmm3);
Serial.print(" ");
Serial.print("t4=");
Serial.print(pwmm4);
Serial.print("  ");





 timePrevp = timep;  // the previous time is stored before the actual time read
    timep = millis();  // actual time read
    elapsedTimep = (timep - timePrevp) / 1000;


 
 
    /*///////////////////////////P I D for Pitch//////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and
*the real measured angle*/
errorp = pitch - desired_angle;
   
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_pp = kpp*errorp;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <errorp <3)
{
  pid_ip = pid_ip+(kip*errorp);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time.
Finnaly we multiply the result by the derivate constant*/

pid_dp = kdp*((errorp - previous_errorp)/elapsedTimep);

/*The final PID values is the sum of each of this 3 parts*/
PIDp = pid_pp + pid_ip + pid_dp;

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PIDp < -400)
{
  PIDp=-400;
}
if(PIDp > 400)
{
  PIDp=400;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmfrontp = throttle - PIDp;
pwmbackp = throttle + PIDp;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmbackp < 1100)
{
  pwmbackp= 1100;
}
if(pwmbackp> 1900)
{
  pwmbackp=1900;
}
//Left
if(pwmfrontp < 1100)
{
  pwmfrontp= 1100;
}
if(pwmfrontp > 1900)
{
  pwmfrontp=1900;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
 Serial.print("pitch:");
  Serial.print(pitch);
  Serial.print("  ");

 
//thruster5.writeMicroseconds(pwmfrontp);
//thruster6.writeMicroseconds(pwmfrontp);

//thruster8.writeMicroseconds(pwmbackp);
//thruster7.writeMicroseconds(pwmbackp);


previous_errorp = errorp; //Remember to store the previous error.
//Serial.print("L1andR1 Servo=");
//Serial.print(pwmfrontp);
//Serial.print(" ");


//Serial.print("L2andR2 Servo=");
//Serial.print(pwmbackp);
//Serial.print(" ");



timePrevr = timer;  // the previous time is stored before the actual time read
    timer = millis();  // actual time read
    elapsedTimer = (timer - timePrevr) / 1000;

Serial.println();

  /*///////////////////////////P I D for ROLL///////////////////////////////////*/


/*First calculate the error between the desired angle and
*the real measured angle*/
errorr = roll - desired_angle;
   
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_pr = kpr*errorr;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <errorr <3)
{
  pid_ir = pid_ir+(kir*errorr);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time.
Finnaly we multiply the result by the derivate constant*/

pid_dr = kdr*((errorr - previous_errorr)/elapsedTimer);

/*The final PID values is the sum of each of this 3 parts*/
PIDr = pid_pr + pid_ir + pid_dr;

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PIDr < -400)
{
  PIDr=-400;
}
if(PIDr > 400)
{
  PIDr=400;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLeftr = throttle + PIDr;
pwmRightr = throttle - PIDr;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRightr < 1100)
{
  pwmRightr= 1100;
}
if(pwmRightr > 1900)
{
  pwmRightr = 1900;
}
//Left
if(pwmLeftr < 1100)
{
  pwmLeftr = 1100;
}
if(pwmLeftr > 1900)
{
  pwmLeftr=1900;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
 Serial.print("roll:");
  Serial.print(roll);
  Serial.print("  ");
//thruster5.writeMicroseconds(pwmLeftr);
//thruster8.writeMicroseconds(pwmLeftr);

//thruster6.writeMicroseconds(pwmRightr);
//thruster7.writeMicroseconds(pwmRightr);

previous_errorr = errorr; //Remember to store the previous error.
//Serial.print("L1,L2 Servo=");
//Serial.print(pwmLeftr);
//Serial.print(" ");
//Serial.print("R1,R2 Servo=");
//Serial.print(pwmRightr);
//Serial.print("   ");


/*//////////////////////////Final Thruster pwm 5,6,7,8////////////////////*/

if (depth_p_pwm>400){
  depth_p_pwm=400;
  }
if (depth_p_pwm<-400){
  depth_p_pwm=-400;
  }

finalpwm5=(pwmfrontp+pwmLeftr)/2 +depth_p_pwm;
finalpwm6=(pwmfrontp+pwmRightr)/2 +depth_p_pwm;
finalpwm7=(pwmbackp+pwmRightr)/2 + depth_p_pwm;
finalpwm8=(pwmfrontp+pwmRightr)/2 +depth_p_pwm;

thruster5.writeMicroseconds(finalpwm5);
thruster6.writeMicroseconds(finalpwm6);
thruster7.writeMicroseconds(finalpwm7);
thruster8.writeMicroseconds(finalpwm8);

int pwmm5 = finalpwm5;
int pwmm6 = finalpwm6;
int pwmm7 = finalpwm7;
int pwmm8 = finalpwm8;

/*
if (depth_p_pwm>400){depth_p_pwm=400;}
if (depth_p_pwm<-400){depth_p_pwm=-400;}
thruster5.writeMicroseconds(1500+depth_p_pwm);
thruster6.writeMicroseconds(1500+depth_p_pwm);
thruster7.writeMicroseconds(1500+depth_p_pwm);
thruster8.writeMicroseconds(1500+depth_p_pwm);
int pwmm5 = 1500+depth_p_pwm;
int pwmm6 = 1500+depth_p_pwm;
int pwmm7 = 1500+depth_p_pwm;
int pwmm8 = 1500+depth_p_pwm;*/
Serial.print("t5=");
Serial.print(pwmm5);
Serial.print(" ");
Serial.print("t6=");
Serial.print(pwmm6);
Serial.print("   ");
Serial.print("t7=");
Serial.print(pwmm7);
Serial.print(" ");
Serial.print("t8=");
Serial.print(pwmm8);
Serial.print("   ");
fd=0;
lr=0;
  }
//end of loop void
