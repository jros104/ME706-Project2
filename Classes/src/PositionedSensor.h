#ifndef PositionedSensor_H
#define PositionedSensor_H

class PositionedSensor {
  private:
	
	Sensor* sensor;
    float x;
    float y;
    float angle;

  public:
    PositionedSensor(Sensor* sensor, float x, float y, float angle){
      this->sensor = sensor;
      this->x = x;
	  this->y = y;
	  this->angle = angle * PI/180.0;
    }

    float getXDistance() {
      float dist = this->sensor->getDistance();
	  float x1 = dist * cos(this->angle);
	  return x + x1;
	 
    }
	
	float getYDistance() {
      float dist = this->sensor->getDistance();
	  float y1 = dist * sin(this->angle);
	  return y + y1;
    }
};
#endif