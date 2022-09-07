//************************************
//         motors.h    
//************************************
#include <Servo.h> 
#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define MIN_SPEED_CAP 1           //Set the minimum speed value that can be written to the motors
#define MAX_SPEED_CAP 1           //Set the maximum speed value that can be written to the motors

extern Servo right_motor;
extern Servo left_motor;
extern Servo Gate_servo;
extern Servo pickup_motor;
extern int encoder_pickup;
extern int limit_switch;
extern bool pickup_calibration_complete;

void check_speed_limits(/*parameters*/);
void DC_motors();


#endif /* MOTORS_H_ */
