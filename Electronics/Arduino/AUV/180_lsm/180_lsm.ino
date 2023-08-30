#include <Wire.h>
#include <LSM303.h>
LSM303 compass;
int first = 0;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.read();
  first = compass.heading();
Serial.print("first reading");
first = compass.heading();
Serial.println(first);
Serial.print("now second reading");
 compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}
void loop() 
{
  compass.read();
 
  int heading = compass.heading();
  int k;
  k = heading - first ;


if( k < 180)

{ k+=180;
k = 180-k;
}
 
  if( k > 180)
{ k-=180;
k = k-180;
}
 
  Serial.println(k);
  delay(100);
}

 
 
