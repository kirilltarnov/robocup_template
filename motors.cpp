#include "motors.h"
#include "Arduino.h"
#include "weight_collection.h"
#include "sensors.h"
bool pickup_calibration_complete = false;
bool can_trigger = true;
int timer = 0;
bool thing = true;
int pickup_state = 0;
int Navigation_State = 0;
int turn_angle = 0;
int limit_switch;
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

  if(io.digitalRead(15) == 1){
    can_trigger = false;
  }
  //monitoring
  Serial.print("Navigation State: ");
  Serial.print(Navigation_State);
  Serial.print(" Left encoder: ");
  Serial.print(Encoder_Left);
  Serial.print(" Right encoder: ");
  Serial.print(Encoder_Right);
  Serial.print(" Turning angle: ");
  Serial.println(turn_angle);

  switch (Navigation_State) {
    case No_move:
        if (io.digitalRead(15) == 1) {
          Navigation_State = Moveforward;
        }
      break;
      
    case Moveforward:
    // if (weight_found) {
    //     right_motor.writeMicroseconds(1500);
    //     left_motor.writeMicroseconds(1500);
    //     Navigation_State = No_move;
    // } else {
    //     right_motor.writeMicroseconds(1950);
    //     left_motor.writeMicroseconds(1050);
    // }
      right_motor.writeMicroseconds(1950);
      left_motor.writeMicroseconds(1950);
      //Checking if there is an obstruction to the left of the robot, turn right
      if ((Left_sensor > 550 && Right_sensor < 550) || pole_ramp_left){
        Navigation_State = TurnRight;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(2000,6000); // determine turn angle
      } else if ((Left_sensor < 550 && Right_sensor > 550) || pole_ramp_right) {
        //Checking if there is an obstruction to the right of the robot, turn left
        Navigation_State = TurnLeft;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(2000,6000); // determine turn angle
      } else if ((Left_sensor > 550 && Right_sensor > 550) || pole_ramp_middle) {
        //Checking if there is an obstruction straight ahead, move back and turn
        Navigation_State = MoveBackward;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(3000,6000); // determine turn angle
      } else {
        right_motor.writeMicroseconds(1950);
        left_motor.writeMicroseconds(1950);
      }
      // if (/*weight_left || */pole_ramp_right) {
      //   //Navigation_State = No_move;
      //   right_motor.writeMicroseconds(1950);
      //   left_motor.writeMicroseconds(1700);
      // } else if (/*weight_right ||*/ pole_ramp_left){
      //   //Navigation_State = No_move;
      //   right_motor.writeMicroseconds(1800);
      //   left_motor.writeMicroseconds(1950);
      // }
      break;

    case MoveBackward:
        right_motor.writeMicroseconds(1050);
        left_motor.writeMicroseconds(1050);
      if ((Encoder_Left < -400 && Encoder_Right < -400)) {
        if (turn_angle > 6000){
          Navigation_State = TurnLeft;
        } else {
          Navigation_State = TurnRight;
        }
        Encoder_Left = 0;
        Encoder_Right = 0;
      }
      break;

    case TurnRight:
      right_motor.writeMicroseconds(1050);
      left_motor.writeMicroseconds(1950);
      if (Encoder_Left > turn_angle && Encoder_Right < -turn_angle) {
        Navigation_State = Moveforward;
      }
      break;

    case TurnLeft:
      right_motor.writeMicroseconds(1950);
      left_motor.writeMicroseconds(1050);
      // if(Encoder_Left < -(12000 - turn_angle) && Encoder_Right > (12000 - turn_angle)) {
      //   Navigation_State = Moveforward;
      // }
      if (Encoder_Left < turn_angle && Encoder_Right > -turn_angle) {
        Navigation_State = Moveforward;
      }
      break;

      
  }
      
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

//  if(Navigation_State == 1){

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
