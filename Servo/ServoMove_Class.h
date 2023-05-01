#ifndef ServoMove_Class_H
# define ServoMove_Class_H

#include <Servo.h>

class ServoMove {
  private:
    Servo servo;
    int pos;
    int step;
    int minPos;
    int maxPos;

  public:
    ServoMove(int servoPin, int stepSize, int minPosition, int maxPosition) {
      servo.attach(servoPin);
      step = stepSize;
      pos = minPosition;
      minPos = minPosition;
      maxPos = maxPosition;
    }

    
    void scan() {
      servo.write(pos);
      Serial.println(servo.);
      delay(20);
      Serial.println(pos);
      pos += step;
      if (pos > 180 || pos < 0) {
        step = -step;
      }
    }
    

    void moveTo(int angle) {
      servo.write(angle);
      pos = angle;
    }
};

#endif