

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int step = 1;

void setup() {
  myservo.attach(8);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
 //angle(80);
 //sweep2();
 myservo.write(0);
 delay(1000);
 myservo.write(90);
 delay(1000);

 Serial.println(pos);
}

void sweep(){
   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  
  // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void angle(int a){

  myservo.write(a);
      pos = a;
}

void sweep2(){
  myservo.write(pos);
  delay(20);
      pos += step;
      if (pos > 180 || pos < 0) {
        step = -step;
      }
    }


