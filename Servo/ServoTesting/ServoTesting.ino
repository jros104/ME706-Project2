
#include <ServoMove_Class.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position
int step = 1;

ServoMove scanner( 8, step,  0,  180);



void setup() {
  
  Serial.begin(9600);
}


void loop() {
 
delay(100);
 scanner.moveTo(0);
 delay(1000);
 scanner.moveTo(90);
 delay(1000);

}

