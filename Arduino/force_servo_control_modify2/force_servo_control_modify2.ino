#include <Servo.h> 
Servo myservo;  
 

int fsrPin = 1;
double fsrReading;
 

void setup() 
{
  myservo.attach(9); 
  Serial.begin(9600); 
}

void loop() 
{ 
  double pos = 80.00; 
  fsrReading = analogRead(fsrPin); 
  double force = (15.311 * exp(0.005199 * fsrReading) * 9.8 * 0.001 - 0.15) * 10; 
  Serial.println(force / 10);
   pos = 80.00 - force * 0.75;
   if(pos<=20)
   pos=20;
   myservo.write(pos);
}
