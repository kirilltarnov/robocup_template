#include "motors.h"
#include "Arduino.h"
#include "weight_collection.h"

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
  pickup_motor.writeMicroseconds(1400);
//  if (encoder_pickup > -1 ) {
//    pickup_motor.writeMicroseconds(1900);
//  } 
//  if (encoder_pickup < -1000) {
//    pickup_motor.writeMicroseconds(1100);
//    delay(1000);
//    pickup_motor.writeMicroseconds(1500);
//  }  
  
  if(State == 1){
    right_motor.writeMicroseconds(1900);
    left_motor.writeMicroseconds(1900);
  }else if (State == 2) {
    right_motor.writeMicroseconds(1100);
    left_motor.writeMicroseconds(1100);
  } else if (State ==3) {
    right_motor.writeMicroseconds(1100);
    left_motor.writeMicroseconds(1900);
  } else if (State ==4) {
    right_motor.writeMicroseconds(1500);
    left_motor.writeMicroseconds(1500);
  }
  check_speed_limits();

}
