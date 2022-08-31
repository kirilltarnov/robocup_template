/************************************
 *        weight_collection.cpp       *
 *************************************/

 /* This is for functions and tasks for
  *  finding and collecting weights  */


#include "weight_collection.h"
#include "Arduino.h"
#include "sensors.h" 

void weight_scan()
{
  if (State == 0) {
    State = 1;
  }else if(State == 1 && (Left_sensor > 550 || Right_sensor > 550)) {
    State = 2;
  }else if(State == 2 && Encoder1 > 500 && Encoder2 > 500 {
    
  }
}

void collect_weight()
{
  /* When ready, collect the weight */
   Serial.println("Collecting weight \n");
}
