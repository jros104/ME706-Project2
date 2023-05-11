#ifndef Defines_H
#define Defines_H


// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

#define NO_READ_GYRO  //Uncomment if GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.


#define PHOTO_L_SHORT_PIN A3
#define PHOTO_R_SHORT_PIN A5
#define PHOTO_R_LONG_PIN A1
#define PHOTO_L_LONG_PIN A2

#define IR_L_LONG_PIN A8
#define IR_R_LONG_PIN A4
#define IR_L_SHORT_PIN A15
#define IR_R_SHORT_PIN A9


#define SONAR_ECHO_PIN 6
#define SONAR_TRIG_PIN 7

#define FAN_PIN 13
#define ON true
#define OFF false


// Fire Detection
#define FIRE_DETECTION_ROTATION_SPEED 0.08
#define FIRE_DETECTION_TOLERANCE 10

// Approach Fire
#define APPROACH_FIRE_DIST 25

#define BASE_SPEED 2

// Obstacle Avoidance
#define AVOID_SPEED BASE_SPEED
#define AVOID_DELAY (125/2.0 * pow(AVOID_SPEED,2) -875 * AVOID_SPEED + 3500)

#endif
