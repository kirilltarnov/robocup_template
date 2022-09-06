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


void DC_motors() {

  if(State == 1){
    right_motor.writeMicroseconds(1950);
    left_motor.writeMicroseconds(1950);
  }else if (State == 2) {
    right_motor.writeMicroseconds(1050);
    left_motor.writeMicroseconds(1050);
  } else if (State == 3) {
    right_motor.writeMicroseconds(1050);
    left_motor.writeMicroseconds(1950);
  } else if (State == 4) {
    right_motor.writeMicroseconds(1500);
    left_motor.writeMicroseconds(1500);
  }
  check_speed_limits();

}
