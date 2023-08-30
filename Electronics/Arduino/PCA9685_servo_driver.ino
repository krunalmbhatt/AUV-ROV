#include <Wire.h>


#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2=Adafruit_PWMServoDriver();

uint8_t servonum = 0;
uint8_t pulselen;
int servomin = 150;
int servomax = 600;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin(); 
   pwm2.begin(); 
   pwm.setPWMFreq(50);
   pwm2.setPWMFreq(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print(servonum);
  pulselen= map(90 , 0 , 180 ,servomin , servomax );
  pwm.setPWM(0,0,pulselen); 
  pwm2.setPWM(1,0,pulselen); 
  
  //servonum++;
  delay(500);
}
