/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep

 modified 6 May 2015
 by Shaun Husain
*/ 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position 

float oneTick = 3.6;
float ninetyDegrees = 90.0/oneTick;
String incomingByte = "";
 
void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(2);  // attaches the servo on pin 2 to the servo object 
  pos = 30+ninetyDegrees*1;
} 
 
void loop() 
{ 
  int i = 1;
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.readString();
  
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte.toInt());
    
    moveTo(incomingByte.toInt()/oneTick);
  }
  
  /*
  for(; i<=4;  i++){
    moveTo(30+ninetyDegrees*i);
    delay(5000);
  }
  for(; i<1;  i--){
    moveTo(30+ninetyDegrees*i);
    delay(5000);
  }
  */
} 

void moveTo(int location){
  if(pos<location){
    for(; pos <= location; pos += 1)
    {                                 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(20);                       // waits 15ms for the servo to reach the position 
    } 
  }
  else{
    for(; location <= pos; pos -= 1)
    {                                 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(20);                       // waits 15ms for the servo to reach the position 
    } 
  }
  
}
