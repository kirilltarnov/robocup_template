#include "motors.h"
#include "Arduino.h"
#include "weight_collection.h"
#include "sensors.h"
bool pickup_calibration_complete = false;
bool can_trigger = true;
int timer = 0;
bool thing = true;
int pickup_state = 0;
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

void DC_motors() {
  //Pick-up motor using a joystick for testing
  
//  if(pickup_mechanism == 0){
//    pickup_motor.writeMicroseconds(1500);
//  } else if(pickup_mechanism == 1) {
//    pickup_motor.writeMicroseconds(1950);
//  } else {
//    pickup_motor.writeMicroseconds(1500);
//  }

//  Calibrate pickup mechanism encoder
//   if(pickup_mechanism == 1 && can_trigger == true) {
//     pickup_calibration_complete = false;
//   }
 // timer = timer + 1;
//  if (pickup_calibration_complete == false) {
//    pickup_motor.writeMicroseconds(PICKUP_REV_SPEED);
//    if (limit_switch == 1) {
//      pickup_calibration_complete = true;
//    }
//  } else if ((low_right_sensor > 350 && can_trigger == true;) {
//    pickup_motor.writeMicroseconds(PICKUP_CAL_SPEED);
//    timer = 0;
//    can_trigger = false;
//  } else if (can_trigger == false && timer > 25) {
//    pickup_motor.writeMicroseconds(1500);
//  }

Serial.print(pickup_state);
switch(pickup_state) {
  case 0:
    pickup_motor.writeMicroseconds(joystick_map_x);
    if (can_trigger == false) {
      pickup_state = 1;
    }
    break;
  case 1:
    if(limit_switch2 == 0) {
      pickup_motor.writeMicroseconds(1050);
    } else {
      pickup_motor.writeMicroseconds(1500);
      pickup_state = 2;
    }
    break;
  case 2:
    if (low_right_sensor > 350) {
      pickup_motor.writeMicroseconds(1950);
      pickup_state = 3;
    }
    break;
  case 3:
    if (limit_switch == HIGH) {
      pickup_state = 1;
      can_trigger = true;
  }
  
}






//if (thing == false) {
//  timer = timer + 1;
//}
//Serial.println(timer);
//if (thing == false && timer > 30) {
//  pickup_motor.writeMicroseconds(1500);
//  thing = true;
//  timer = 0;
//}


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
//  if(Navigation_State == 1){
//    right_motor.writeMicroseconds(1950);
//    left_motor.writeMicroseconds(1950);
//  }else if (Navigation_State == 2) {
//    right_motor.writeMicroseconds(1050);
//    left_motor.writeMicroseconds(1050);
////    right_motor.writeMicroseconds(1500);
////    left_motor.writeMicroseconds(1500);
//  } else if (Navigation_State == 3) {
//    right_motor.writeMicroseconds(1050);
//    left_motor.writeMicroseconds(1950);
//  } else if (Navigation_State == 4) {
//    right_motor.writeMicroseconds(1950);
//    left_motor.writeMicroseconds(1050);
//  }


}
