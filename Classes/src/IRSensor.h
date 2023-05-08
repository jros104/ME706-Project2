#ifndef IRSensor_H
#define IRSensor_H

class IRSensor : public Sensor {
  private:
  
    float _coeff1;
    float _coeff2;
    float _minDistance;
    float _maxDistance;

  public:
    IRSensor(int pin, float x, float y, float angle, float coeff1, float coeff2, float minDistance, float maxDistance) : Sensor(pin, x, y, angle) {
      pinMode(pin, INPUT);

      this->_coeff1 = coeff1;
      this->_coeff2 = coeff2;
      this->_minDistance = minDistance;
      this->_maxDistance = maxDistance;
    }

    float getDistance() {
      int sensorValue = analogRead(pin);
      float distance = this->_coeff1 * (pow(sensorValue, this->_coeff2));

      if (distance > this->_maxDistance) {
        distance = this->_maxDistance;
      } else if (distance < this->_minDistance) {
        distance = this->_minDistance;
      }

      return distance;
    }
};
#endif