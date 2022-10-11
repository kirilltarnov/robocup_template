//************************************
//         motors.h    
//************************************
#include <Servo.h> 
#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define MIN_SPEED_CAP 1          //Set the minimum speed value that can be written to the motors
#define MAX_SPEED_CAP 1           //Set the maximum speed value that can be written to the motors
#define PICKUP_CAL_SPEED 1950
#define PICKUP_REV_SPEED 1050
#define PICKUP_FOR_SPEED 1900
#define PICKUP_ENCODER_DIST_OUT 1320
#define PICKUP_ENCODER_DIST_IN 500
#define MOTOR_STOP 1500

extern Servo right_motor;
extern Servo left_motor;
extern Servo Gate_servo;
extern Servo pickup_motor;
extern int encoder_pickup;
extern int limit_switch;
extern bool pickup_calibration_complete;
extern int joystick_map_x;
extern bool can_trigger;

void check_speed_limits(/*parameters*/);
void DC_motors();


#endif /* MOTORS_H_ */
