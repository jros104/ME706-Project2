#include <Servo.h>  //Need for Servo pulse output
#define MAX(A, B, C, D) ((abs(A) > abs(B) ? A : B) > abs(C) ? (abs(A) > abs(B) ? A : B) : C) > abs(D) ? ((abs(A) > abs(B) ? A : B) > C ? (abs(A) > abs(B) ? A : B) : C) : D

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


const float wheel_radius = 0.0275;
const float length = 0.075;
const float width = 0.09;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29


float max_omega = 3.267;

float ratio = 500.0 / max_omega;

void wheel_kinematics(float vel_x, float vel_y, float omega_z){
  // Inputs in cm/s
  // Convert to m/s

  vel_x = -vel_x / 100.0;
  vel_y = vel_y / 100.0;

  float omega_FL = (1 / wheel_radius) * ((vel_x) + (vel_y) + (-(length + width) * omega_z));
  float omega_FR = (1 / wheel_radius) * ((vel_x) + (-vel_y) + ((length + width) * omega_z));
  float omega_BL = (1 / wheel_radius) * ((vel_x) + (-vel_y) + (-(length + width) * omega_z));
  float omega_BR = (1 / wheel_radius) * ((vel_x) + (vel_y) + ((length + width) * omega_z));

  if ((abs(omega_FL) > max_omega) || (abs(omega_FR) > max_omega) || (abs(omega_BL) > max_omega) || (abs(omega_BR) > max_omega)){
    float highestWheelSpeed = MAX(omega_FL, omega_FR, omega_BL, omega_BR);
    float scaleSpeeds = max_omega / highestWheelSpeed;

    omega_FL *= scaleSpeeds;
    omega_FR *= scaleSpeeds;
    omega_BL *= scaleSpeeds;
    omega_BR *= scaleSpeeds;
  }

  omega_FL *= ratio;
  omega_FR *= ratio;
  omega_BL *= ratio;
  omega_BR *= ratio;

  left_font_motor.writeMicroseconds(1500 + omega_FL);
  left_rear_motor.writeMicroseconds(1500 + omega_BL);
  right_rear_motor.writeMicroseconds(1500 - omega_BR);
  right_font_motor.writeMicroseconds(1500 - omega_FR);
}

float saturate(float value, float saturationLimit){
  return value > saturationLimit ? saturationLimit : value < -saturationLimit ? -saturationLimit : value;
}

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

