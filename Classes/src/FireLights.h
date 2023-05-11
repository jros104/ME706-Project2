#ifndef FireLights_H
#define FireLights_H

class FireLights {
public:
  FireLights(int pin1, int pin2, int interval) : led1Pin(pin1), led2Pin(pin2), flashInterval(interval) {}

  void begin() {
    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    previousMillis = 0;
    ledState = LOW;
  }

  void update() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= flashInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(led1Pin, ledState);
      digitalWrite(led2Pin, !ledState);
    }
  }

private:
  int led1Pin, led2Pin;
  int flashInterval;
  unsigned long previousMillis;
  int ledState;
};



#endif