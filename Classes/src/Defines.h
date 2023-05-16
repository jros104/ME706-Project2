#ifndef Defines_H
#define Defines_H


// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

#define NO_READ_GYRO  //Uncomment if GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.
#define USE_KALMAN true

#define PHOTO_L_SHORT_PIN A5
#define PHOTO_R_SHORT_PIN A4
#define PHOTO_R_LONG_PIN A7
#define PHOTO_L_LONG_PIN A6

#define IR_L_LONG_PIN A10
#define IR_R_LONG_PIN A11
#define IR_L_SHORT_PIN A15
#define IR_R_SHORT_PIN A14


#define SONAR_ECHO_PIN 6
#define SONAR_TRIG_PIN 7

#define FAN_PIN 26
#define ON false
#define OFF true


// Fire Detection
#define FIRE_DETECTION_ROTATION_SPEED 0.1
#define FIRE_DETECTION_TOLERANCE 50

// Approach Fire
#define APPROACH_FIRE_DIST 25

#define BASE_SPEED 2

// Obstacle Avoidance
#define OBSTIK_DIST 8
#define OBSTIK_DIST_EXTRA 3
#define OBSTIK_MIN_SPACE 25
#define OBSTIK_CLEAR 20
#define AVOID_SPEED BASE_SPEED
#define AVOID_DELAY_SIDEWAYS 300
#define AVOID_DELAY_FORWARDS 1300

#endif
