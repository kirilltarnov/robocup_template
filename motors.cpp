#include "motors.h"
#include "Arduino.h"
#include "sensors.h"
bool pickup_calibration_complete = false;
bool can_trigger = true;
int search_timer = 0;
bool thing = true;
int pickup_state = 0;
int Navigation_State = 0;
int turn_angle = 0;
int limit_switch_inner;
int jam_timer = 0;
int decel = 0;
int turning_timer = 0;
uint8_t pick_up_counter = 0;
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

void navigation() {

  if(io.digitalRead(15) == 1){
    can_trigger = false;
  }


  //monitoring
//  Serial.print("Navigation State: ");
//  Serial.print(Navigation_State);
//  Serial.print(" Left encoder: ");
//  Serial.print(Encoder_Left);
//  Serial.print(" Right encoder: ");
//  Serial.print(Encoder_Right);
//  Serial.print(" Turning angle: ");
//  Serial.println(turn_angle);

  switch (Navigation_State) {
    case No_move:
              right_motor.writeMicroseconds(1500);
          left_motor.writeMicroseconds(1500); 
      if (io.digitalRead(15) == 1) {
        Navigation_State = Moveforward;
      }
      break;
      
    case Moveforward:
      if ((turning_timer > 40) && (Navigation_State != Deceleration)) {
        Navigation_State = TurnOnSpot; 
        Encoder_Left = 0;
        Encoder_Right = 0; 
        turn_angle = 15000; // determine turn angle
      } else {
        turning_timer += 1;
      }
      right_motor.writeMicroseconds(1950);
      left_motor.writeMicroseconds(1950);
      
      //Checking if there is an obstruction to the left of the robot, turn right
      if ((Left_sensor > 550 && Right_sensor < 550) && (!US_right_wall_too_close)){
        Navigation_State = TurnRight;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(500,3000); // determine turn angle
      } else if ((Left_sensor < 550 && Right_sensor > 550) && (!US_left_wall_too_close)) {
        //Checking if there is an obstruction to the right of the robot, turn left
        Navigation_State = TurnLeft;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(500,3000); // determine turn angle
      } else if ((Left_sensor > 550 && Right_sensor > 550) || pole_ramp_middle) {
        //Checking if there is an obstruction straight ahead, move back and turn
        Navigation_State = MoveBackward;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(1000,3000); // determine turn angle
      } else  if (pole_ramp_left) {
        Navigation_State = BankRight;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(1000,3000); // determine turn angle
      } else if (pole_ramp_right) {
        Navigation_State = BankLeft;
        Encoder_Left = 0; //reset encoder values 
        Encoder_Right = 0;
        turn_angle = random(1000,3000); // determine turn angle
      } else {
        right_motor.writeMicroseconds(1950);
        left_motor.writeMicroseconds(1950);
      }

      break;

    case MoveBackward:
        right_motor.writeMicroseconds(1050);
        left_motor.writeMicroseconds(1050);
      if ((Encoder_Left < -400 && Encoder_Right < -400)) {
        if ((turn_angle > 6000) && (!US_left_wall_too_close)){
          Navigation_State = TurnLeft;
        } else if (!US_right_wall_too_close) {
          Navigation_State = TurnRight;
        } else {
          //Turn 180 deg
          turn_angle = 7500;
          Navigation_State = TurnRight;
        }
        Encoder_Left = 0;
        Encoder_Right = 0;
      }
      break;

    case TurnRight:
      right_motor.writeMicroseconds(1050);
      left_motor.writeMicroseconds(1950);
      if ((Encoder_Left > turn_angle && Encoder_Right < -turn_angle)) {
        Navigation_State = Moveforward;
      }
      break;

    case TurnLeft:
      right_motor.writeMicroseconds(1950);
      left_motor.writeMicroseconds(1050);
      // if(Encoder_Left < -(12000 - turn_angle) && Encoder_Right > (12000 - turn_angle)) {
      //   Navigation_State = Moveforward;
      // }
      if ((Encoder_Left < turn_angle && Encoder_Right > -turn_angle)) {
        Navigation_State = Moveforward;
      }
      break;

      //Bank turns at a smaller angle compared to 'Turn'
      case BankLeft:
        right_motor.writeMicroseconds(1900);
        left_motor.writeMicroseconds(1100);
        if (Encoder_Left < turn_angle && Encoder_Right > -turn_angle) {
          Navigation_State = Moveforward;
        }
        break;

      case BankRight:
        right_motor.writeMicroseconds(1100);
        left_motor.writeMicroseconds(1900);
        if (Encoder_Left < turn_angle && Encoder_Right > -turn_angle) {
          Navigation_State = Moveforward;
        }
        break; 
      case Deceleration:
        if (decel < 90) {
          decel += 10;
          right_motor.writeMicroseconds(1950 - decel);
          left_motor.writeMicroseconds(1950 - decel);
        } else {
          right_motor.writeMicroseconds(1500);
          left_motor.writeMicroseconds(1500); 
          if (limit_switch_outer == 1) {
            Navigation_State = Moveforward;
          }
        }
        break;
      case TurnOnSpot:
        right_motor.writeMicroseconds(1950);
        left_motor.writeMicroseconds(1050);
        if ((Encoder_Right > turn_angle && Encoder_Left < -turn_angle) || (weight_found)) {
          turning_timer = 0;
          Encoder_Right = 0;
          Encoder_Left = 0;
          turn_angle = 100;
          Navigation_State = TurnRight;
        }
        break;
  
 }

//  Serial.print("pick_up state: ");
//  Serial.print(pickup_state);
//  Serial.print(" Decel :");
//  Serial.println(decel);
  // Serial.println(low_right_sensor);
//  Serial.println(jam_timer);
}
void pickup() {
  switch(pickup_state) {
    case 0:
    //Use joystick before Start PB is pressed
      pickup_motor.writeMicroseconds(joystick_map_x);
      if (can_trigger == false) {
        pickup_state = 1;
      }
      
      break;
    //Move outward
    case 1:
      if(limit_switch_outer == 0) {
        pickup_motor.writeMicroseconds(1050);
      } else {
        pickup_motor.writeMicroseconds(1500);
        pickup_state = 2;
      }
      break;
    //If weight detected, move inward
    case 2:
      if (low_right_sensor > 350) {
        if (pick_up_counter > 2) {
          pickup_state = 1;
        } else {
          pickup_state = 3;
          jam_timer = 0;
          decel = 0; 
        }

      }
      break;
    case 3:                
      Navigation_State = Deceleration;
      pickup_motor.writeMicroseconds(1950);
      if (jam_timer > 27) {
        Encoder_Left = 0;
        Encoder_Right = 0;
        pickup_state = 1;
        Navigation_State = MoveBackward;
      }
      jam_timer += 1;
      if (limit_switch_inner == HIGH) {
        pick_up_counter += 1;
        pickup_state = 1;
        can_trigger = true;
      }  
      break; 
  }



}
