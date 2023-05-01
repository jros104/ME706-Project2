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

    void calibratePos(){
      int midPos = (minPos + maxPos) / 2; // find the midpoint of the range of motion
      servo.write(midPos); // move the servo to the midpoint
      delay(500); // wait for the servo to settle
      pos = midPos; // set the initial position of the servo
    }

    void scan() {
      servo.write(pos);
      pos += step;
      if (pos > maxPos || pos < minPos) {
        step = -step;
      }
    }

    void moveTo(int angle) {
      if (angle < minPos) {
        angle = minPos;
      }
      else if (angle > maxPos) {
        angle = maxPos;
      }
      servo.write(angle);
      pos = angle;
    }
};
