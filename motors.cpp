#include "motors.h"
#include "Arduino.h"
#include "weight_collection.h"

bool pickup_calibration_complete = false;
bool can_trigger = true;
/* Check whether the speed value to be written is within the maximum
 *  and minimum speed caps. Act accordingly.
 *
 */
void check_speed_limits(/*parameters*/) {
  //Serial.println("Check the motor speed limit \n");
}


/* In this section, the motor speeds should be updated/written.
 *It is also a good idea to check whether value to write is valid.
 *It is also a good idea to do so atomically!
 */
//Pick-up motor values (looking at the motor front on):
//1500-1950 = anticlockwise
//1050-1500 = clockwise
//Encoder values:
//-1760 = OPEN
//+1700 = CLOSED

void DC_motors() {
  //Calibrate pickup mechanism encoder
  if(pickup_mechanism == 1 && can_trigger == true) {
    pickup_calibration_complete = false;
  }
//  
//  if (pickup_calibration_complete == false) {
//    can_trigger = false;
//    pickup_motor.writeMicroseconds(PICKUP_CAL_SPEED);
//    if (limit_switch == 1) {
//      pickup_motor.writeMicroseconds(MOTOR_STOP);
//      pickup_calibration_complete = true;
//      encoder_pickup = 0;
//    }
//  } else {
//    Serial.println("thing");
//    pickup_motor.writeMicroseconds(PICKUP_REV_SPEED);
//    if (encoder_pickup > PICKUP_ENCODER_DIST_OUT) {
//      pickup_motor.writeMicroseconds(MOTOR_STOP);
//      can_trigger = true;
//    } else {
//    }
//    
//  }


//  if (encoder_pickup > -1 ) {
//    pickup_motor.writeMicroseconds(1900);
//  } 
//  if (encoder_pickup < -1000) {
//    pickup_motor.writeMicroseconds(1100);
//    delay(1000);
//    pickup_motor.writeMicroseconds(1500);
//  }  
  if(Navigation_State == 1){
    right_motor.writeMicroseconds(1950);
    left_motor.writeMicroseconds(1950);
  }else if (Navigation_State == 2) {
//    right_motor.writeMicroseconds(1050);
//    left_motor.writeMicroseconds(1050);
    right_motor.writeMicroseconds(1500);
    left_motor.writeMicroseconds(1500);
  } else if (Navigation_State == 3) {
    right_motor.writeMicroseconds(1050);
    left_motor.writeMicroseconds(1950);
  } else if (Navigation_State == 4) {
    right_motor.writeMicroseconds(1950);
    left_motor.writeMicroseconds(1050);
  }

}
