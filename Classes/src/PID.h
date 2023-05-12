// ####################################################################################################################
// ||                       _____  _____  _____         _____  _                  _____  _____                       ||
// ||                      |  __ \|_   _||  __ \       / ____|| |         /\     / ____|/ ____|                      ||
// ||                      | |__) | | |  | |  | |     | |     | |        /  \   | (___ | (___                        ||
// ||                      |  ___/  | |  | |  | |     | |     | |       / /\ \   \___ \ \___ \                       ||
// ||                      | |     _| |_ | |__| |     | |____ | |____  / ____ \  ____) |____) |                      ||
// ||                      |_|    |_____||_____/       \_____||______|/_/    \_\|_____/|_____/                       ||
// ||                                                                                                                ||
// ####################################################################################################################

#ifndef PID_H
#define PID_H

// ######################################################### PID CLASS #########################################################
// Can be instantiated with PID values and its CalculateEffort can be called
// by passing the current error, the saturation limit for the P effort, and dt
// dt --> is the time between the last CalculateEffort call, and the current time
class PIDController{
  private:
    // Variables
    float Kp;
    float Ki;
    float Kd;
    
    float sumError;
    float prevError;
    float prevTime = -1;
    float dt;
    float P, I, D;
  public:
    // Contructor
    PIDController(float Kp, float Ki, float Kd){
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;

      this->sumError = 0;
      this->prevError = 0;
      this->P = 0;
      this->I = 0;
      this->D = 0;
    }

    // Function to calculate the control effort
    float CalculateEffort(float error, float saturationLimit){
      if(prevTime == -1){
        dt = 0;
        this->prevTime = millis();
      } else {
        dt = millis()-prevTime;
      }
      // Proportional Control Effort
      P = Kp * error;
      int sign = (P > 0) - (P < 0);
      // Saturating the Proportional Control Effort
      P = P > saturationLimit ? saturationLimit : P < -saturationLimit ? -saturationLimit : P;

      // Anti Integral windup
      if (abs(P) < saturationLimit){
        sumError += (error+prevError)/2 * dt;  
      }
      // Integral Control Effort
      I = Ki * sumError;

      // Derivative Control Effort
      if (dt != 0){
        D = Kd * ((error - prevError) / dt);
      }

      //Serial.println(sumError);
      
      // Updating the previous Error
      prevError = error;

      float u = P + I + D;
      Serial.println(error);

      u = abs(u) > saturationLimit ? saturationLimit * ((u > 0) - (u < 0)) : u;
      // Returning the total control effort
      return u;
    }

    // Function to reset sum and previous error
    void ResetPID(){
      this->sumError = 0;
      this->prevError = 0;
    }
};

#endif
