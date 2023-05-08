#ifndef MotorFunctions_H
#define MotorFunctions_H
 

extern const byte left_front = 46;
extern const byte left_rear = 47;
extern const byte right_rear = 50;
extern const byte right_front = 51;

extern const float wheel_radius = 2.75;
extern const float length = 7.5;
extern const float width = 9.0;


void wheel_kinematics(float vel_x, float vel_y, float omega_z);

void disable_motors();
void enable_motors();

void stop();

void forward(int speed_val);
void reverse (int speed_val);

void ccw (int speed_val);
void cw (int speed_val);

void strafe_left (int speed_val);
void strafe_right (int speed_val);

void diag_forward_right (int speed_val);
void diag_forward_left (int speed_val);
void diag_backward_left (int speed_val);
void diag_backward_right (int speed_val);

#endif