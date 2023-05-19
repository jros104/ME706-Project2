#ifndef Kalman_H
#define Kalman_H

class Kalman {
  private:
    float processNoise;
    float measurementNoise;
    float x;
    float p;

  public:
	Kalman(){
	  this->processNoise = 1;
      this->measurementNoise = 5;
      this->x = 0;
      this->p = 0;
	}
	
  
    Kalman(float processNoise, float measurementNoise, float initialX, float initialP) {
      this->processNoise = processNoise;
      this->measurementNoise = measurementNoise;
      this->x = initialX;
      this->p = initialP;
    }

    float updateEstimate(float measurement) {
      // time update
      float xPredicted = x;
      float pPredicted = p + processNoise;

      // measurement update
      float kalmanGain = pPredicted / (pPredicted + measurementNoise);
      x = xPredicted + kalmanGain * (measurement - xPredicted);
      p = (1 - kalmanGain) * pPredicted;

      return x;
    }

	void setFirstEstimate(float estimate){
      this->x= estimate;
    }

};
#endif