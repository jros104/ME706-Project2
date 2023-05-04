#ifndef Sensor_H
#define Sensor_H

class Sensor{
  public:
    int pin;
  public:
  
	Sensor(){
      this->pin = 8;
    }
    
    Sensor(int pin){
      this->pin = pin;
    }
	
    virtual float getDistance() {
      // Implementation goes here
    }
};

#endif