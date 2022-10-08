//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.


#include "sensors.h"
#include "Arduino.h"

// Local definitions
#define encoder1PinA 2
#define encoder1PinB 3
#define encoder2PinA 4
#define encoder2PinB 5
int low_right_sensor;
// Read ultrasonic value
void read_ultrasonic(/* Parameters */){
  Serial.println("Ultrasonic value \n");
}

// Read infrared value
void read_infrared(){
  Left_sensor = analogRead(A9);  
  Right_sensor = analogRead(A8); 
  low_right_sensor = analogRead(A7);  
//  Serial.print("Right Sensor: ");
//  Serial.print(Right_sensor);
//  Serial.print(" Left Sensor: ");
//  Serial.print(Left_sensor);
//  Serial.print(" Left Sensor: ");
//  Serial.print(low_right_sensor);
//  Serial.println();
}


// Read colour sensor value
void read_colour(/* Parameters */){
  Serial.println("colour value \n");  
}

void read_limit() {
    limit_switch = digitalRead(20) == HIGH;
}

int buffer1[4][4][10];
int averageinput[4][4];
int head = 1;
int tail = 9;

void circ_buffer_add(int input[4][4]) {
 for (int i =0; i < 4; i++) {
  for (int j =0; j < 4; j++) {
    buffer1[i][j][tail] = input[i][j];
  }
 }
 if (tail == 9) {
  tail = 0;
 } else {
  tail = tail+1;
 }
 if (head == 9) {
  head = 0;
 } else {
  head = head + 1;
 }
}
 void average_Buffer() {
   for (int i =0; i < 4; i++) {
  for (int j =0; j < 4; j++) {
    averageinput[i][j] = 0;
  }
 }
 for (int i =0; i < 4; i++) {
  for (int j =0; j < 4; j++) {
    for (int z =0; z <10; z++) {
      averageinput[i][j] = buffer1[i][j][z] + averageinput[i][j];
    }
  }
  }
  for (int i =0; i < 4; i++) {
  for (int j =0; j < 4; j++) {
    averageinput[i][j] = averageinput[i][j]/10;
  }
 }
 }



// Pass in data and average the lot
void sensor_average(/* Parameters */){
  Serial.println("Averaging the sensors \n");
}
