#ifndef Sonar_H
#define Sonar_H


class Sonar : public Sensor {

private:
  int trigPin;
  int echoPin;

public:
  Sonar(int Tpin, int Epin, float x, float y, float angle)
    : Sensor(pin, x, y, angle),

      this->trigPin = Tpin;
      this->echoPin = Epin;
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);

  float measureDistance() {

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.034 / 2.0;

    if (distance > 400) {
      distance = -1;
    }

    return distance;
  }

  void printDistance(float distance) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(500);
  }
};

#endif