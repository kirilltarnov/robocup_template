/************************************
 *        weight_collection.cpp       *
 *************************************/

 /* This is for functions and tasks for
  *  finding and collecting weights  */


#include "weight_collection.h"
#include "Arduino.h"
#include "sensors.h" 

int turn_angle = 0;

void weight_scan()
{
    //Serial.print(io.digitalRead(15));
  if (State == 0 && io.digitalRead(15) == 1) {
    State = 1;
  }else if(State == 1 && (Left_sensor > 550 || Right_sensor > 550)) {
    State = 2;
    Encoder_Left = 0;
    Encoder_Right = 0;
    turn_angle = random(3000,9000);
  }else if(State == 2 && Encoder_Left < -500 && Encoder_Right < -500) {
    State = 3;
    Encoder_Left = 0;
    Encoder_Right = 0;
  } else if(State == 3 && Encoder_Left > turn_angle && Encoder_Right < -turn_angle) {
    State = 1;
  }
//  Serial.print("Left_Encoder:");
//  Serial.print(Encoder_Left);
//  Serial.print(" Right_Encoder:");
//  Serial.print(Encoder_Right);
//  Serial.print(" State:");
//  Serial.print(State);
//  Serial.println();
}

void collect_weight()
{
  /* When ready, collect the weight */
   Serial.println("Collecting weight \n");
}
