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

void check_speed_limits(/*parameters*/);
void set_motors();


#endif /* MOTORS_H_ */
