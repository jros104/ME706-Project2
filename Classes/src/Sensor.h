#ifndef Sensor_H
#define Sensor_H

class Sensor{
  public:
    int pin;
    float x;
    float y;
    float angle;
  public:
  
	  Sensor(){
      this->pin = 8;
    }
    
    Sensor(int pin, float x, float y, float angle){
      this->pin = pin;
      this->x = x;
      this->y = y;
      this->angle = angle;
    }
	
    virtual float getDistance() {
      // Implementation goes here
    }

    float getXDistance() {
      float dist = this->getDistance();
	  float x1 = dist * cos(this->angle);
	  return x + x1;
    }
	
	float getYDistance() {
      float dist = this->getDistance();
	  float y1 = dist * sin(this->angle);
	  return y + y1;
    }
};

#endif