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
  
  
  float GetDistancePhoto()
  {
    float analog_voltage = analogRead(this->analog_pin) * VCC / 1023.0;
    float estimated_dist = 3; // Calculated through calibration curve
    
    return estimated_dist;    
  }
  
  bool IsLightDetected()
  {
    return analogRead(this->analog_pin) > this->threshold ? true : false;
  }
  
private:
  int analog_pin;
  float threshold;
  float coeff1;
  float coeff2;
  float VCC;
  
};

#endif