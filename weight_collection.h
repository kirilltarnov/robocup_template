
/************************************
 *        weight_collection.h       *
 *************************************/

 /* This header is for functions and tasks for
  *  finding and collecting weights  */

#ifndef WEIGHT_COLLECTION_H_
#define WEIGHT_COLLECTION_H_

#include <Servo.h>                  //control the DC motors
//#include <Herkulex.h>               //smart servo
#include <Wire.h>                   //for I2C and SPI

extern int Navigation_State;
extern int Overall_State;
              //will need sensor library to detect weights


//states for swapping between searching and collecting
#define NO_WEIGHT               0   
#define WEIGHT_FOUND            1

//States for Overall State machine 
#define Startup                 0
#define Navigation_algorithim   1
#define weight_pickup           2
#define Return_tobase           3

//States for Navigation State machine 
#define Moveforward             1
#define MoveBackward            2
#define TurnRight               3
#define TurnLeft                4
void State_machine();


void collect_weight();



#endif /* WEIGHT_COLLECTION_H_ */
