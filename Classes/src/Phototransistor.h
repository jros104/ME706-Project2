#ifndef PHOTO
#define PHOTO 

class Phototransistor
{
public:
  
  Phototransistor(int analog_pin, float coeff1, float coeff2, float threshold) 
    : analog_pin(analog_pin), coeff1(coeff1), coeff2(coeff2), VCC(5.0), threshold(threshold)
  {
    pinMode(analog_pin, INPUT);
  }
  
  
  float getDistance()
  {
    int sensorValue = analogRead(this->analog_pin);
    float distance = this->coeff1 * (pow(sensorValue, this->coeff2));
    
    return distance > 999 ? 999: distance;    
  }
  
	float getAnalog(){
		return analogRead(this->analog_pin);
	}
  
  bool IsLightDetected()
  {
    return analogRead(this->analog_pin) > this->threshold;
  }
  
private:
  int analog_pin;
  float threshold;
  float coeff1;
  float coeff2;
  float VCC;
  
};

#endif