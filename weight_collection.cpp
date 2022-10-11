/************************************
 *        weight_collection.cpp       *
 *************************************/

 /* This is for functions and tasks for
  *  finding and collecting weights  */


//#include "weight_collection.h"
//#include "Arduino.h"
//#include "motors.h"
//#include "sensors.h" 
//
//int turn_angle = 0;
//int Overall_State = weight_pickup;
//int Navigation_State = 0;
//int pickup_mechanism = No_weight;
//int limit_switch;
//void State_machine()
//{
//  if(io.digitalRead(15) == 1){
//    can_trigger = false;
//    Serial.println(1);
//  }
//    //Serial.print(io.digitalRead(15));
////  switch(Overall_State) {
////    case Startup:
////      if(io.digitalRead(15) == 1){
////         pickup_calibration_complete = false;
////        Overall_State = Navigation_algorithim;
////        Navigation_State = Moveforward;
////      }
////      break;
////    case Navigation_algorithim:
////      if (weight_detect) {
////        
////      }
////      switch (Navigation_State) {
////        case Moveforward:
////          if (Left_sensor > 550 || Right_sensor > 550) {
////            Navigation_State = MoveBackward;
////            Encoder_Left = 0; //reset encoder values 
////            Encoder_Right = 0;
////            turn_angle = random(3000,9000); // determine turn angle
////          }
////          break;
////        case MoveBackward:
////          if (Encoder_Left < -500 && Encoder_Right < -500) {
////            if (turn_angle > 6000){
////              Navigation_State = 4;
////            } else {
////              Navigation_State = 3;
////            }
////            Encoder_Left = 0;
////            Encoder_Right = 0;
////          }
////          break;
////        case TurnRight:
////          if (Encoder_Left > turn_angle && Encoder_Right < -turn_angle) {
////            Navigation_State = Moveforward;
////          }
////          break;
////        case TurnLeft:
////          if(Encoder_Left < -(12000 - turn_angle) && Encoder_Right > (12000 - turn_angle)) {
////            Navigation_State = 1;
////          }
////          break;
////      }
////      break;
////    case weight_pickup:
////      switch (pickup_mechanism) {
////        case No_weight:
////          if (low_right_sensor > 350) {
////              pickup_mechanism = Weight;
////          }
////          break;
////        case Weight:
////          if (limit_switch == 1) {
////            pickup_mechanism = reseting;
////          }
////          break;
////        case reseting:
////
////
////          break;
////        case jamming:
////
////
////          break;
////      }
////      break;   
////    case Return_tobase:
////    // code block
////      break;
////    
////}
//  //Serial.println(pickup_mechanism);
//
////  Serial.print("Left_Encoder:");
////  Serial.print(Encoder_Left);
////  Serial.print(" Right_Encoder:");
////  Serial.print(Encoder_Right);
////  Serial.print(" Navigation_State:");
////  Serial.print(Navigation_State);
////  Serial.println();
//}
//
////void Jamming() {
////  pickup_mechanism = jamming;
////}
//
//void collect_weight()
//{
//  /* When ready, collect the weight */
//   Serial.println("Collecting weight \n");
//}
