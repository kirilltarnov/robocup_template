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
bool limit_switch_outer;
int low_right_sensor;
bool pole_ramp_found = false;
// int8_t counter_average = 0;
double weighted_sum = 0;
double raw_sum = 0;
double residual = 0;
boolean weight_found = false;
bool pole_ramp_middle = false;
bool pole_ramp_left = false;
bool pole_ramp_right = false;
bool weight_left = false;
bool weight_right = false;
bool weight_middle = false;
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
int VL53_raw_matrix [8][6]; //16 int array to hold previous time-step data
int VL53_weighted_matrix [8][6]; //16 int array to hold previous time-step data
int CentiA;
int CentiB;

// Read ultrasonic value
void read_ultrasonic(/* Parameters */){
    digitalWrite(32, LOW);
    digitalWrite(30, LOW);
  delayMicroseconds(2);
  digitalWrite(32, HIGH);
  digitalWrite(30, HIGH);
  delayMicroseconds(10);
  digitalWrite(32, LOW);
  digitalWrite(30, LOW);
    CentiA = pulseIn(33, HIGH);
    CentiB =  pulseIn(31, HIGH);
  
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
    limit_switch_inner = digitalRead(20);
    limit_switch_outer = digitalRead(14);
}

int buffer1[4][4][10];
int averageinput[4][4];
int head = 1;
int tail = 9;

//void circ_buffer_add(int input[4][4]) {
// for (int i =0; i < 4; i++) {
//  for (int j =0; j < 4; j++) {
//    buffer1[i][j][tail] = input[i][j];
//  }
// }
// if (tail == 9) {
//  tail = 0;
// } else {
//  tail = tail+1;
// }
// if (head == 9) {
//  head = 0;
// } else {
//  head = head + 1;
// }
//}
// void average_Buffer() {
//  for (int i =0; i < 4; i++) {
//    for (int j =0; j < 4; j++) {
//      averageinput[i][j] = 0;
//    }
//  }  
//  for (int i =0; i < 4; i++) {
//    for (int j =0; j < 4; j++) {
//      for (int z =0; z <10; z++) {
//        averageinput[i][j] = buffer1[i][j][z] + averageinput[i][j];
//      }
//    }
//  }
//  for (int i =0; i < 4; i++) {
//    for (int j =0; j < 4; j++) {
//
//      averageinput[i][j] = (round(averageinput[i][j]/100))*10;
//    }
//  }
// }



// Pass in data and average the lot
void sensor_average(/* Parameters */){
  pole_ramp_middle = false;
  pole_ramp_left = false;
  pole_ramp_right = false;
  weight_left = false;
  weight_right = false;
  weight_middle = false;
  weight_found = false;
  pole_ramp_found = false;

  //Poll sensor for new data (ToF)
 if (myImager.isDataReady() == true)
 {
   if (myImager.getRangingData(&measurementData)) //Read distance data into array
   {
      
     //The ST library returns the data transposed from zone mapping shown in datasheet
     //Pretty-print data with increasing y, decreasing x to reflect reality
     for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
     {
       for (int x = imageWidth - 3 ; x >= 0 ; x--)
       {
        //Serial.print("\t");
        // take the raw data from the VL sensor, round it to a number of 10
        //  measurement_rounded = measurementData.distance_mm[x+y]/10;
        //  measurement_rounded = round(measurement_rounded)*10;
        //  measurement_rounded = int(measurement_rounded); //convert from double to int
        VL53_raw_matrix[y/imageWidth][x] = measurementData.distance_mm[x+y]; //place rounded data in a matrix 
        if (x == 5) {
          VL53_raw_matrix[y/imageWidth][x] -= 490;
        } else if (x == 4) {
          VL53_raw_matrix[y/imageWidth][x] -= 400;
        } else if (x == 3) {
          VL53_raw_matrix[y/imageWidth][x] -= 340;
        } else if (x == 2) {
          VL53_raw_matrix[y/imageWidth][x] -= 300;
        } else if (x == 1) {
          VL53_raw_matrix[y/imageWidth][x] -= 280;
        } else if (x == 0) {
          VL53_raw_matrix[y/imageWidth][x] -= 250;
        } 
        if (y == imageWidth*7) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * -1;
        } else if (y == imageWidth*6) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * -0.5;
        } else if (y == imageWidth*5) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * -0.25;
        } else if (y == imageWidth*4) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * 0;
        } else if (y == imageWidth*3) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * 0;
        } else if (y == imageWidth*2) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * 0.25;
        } else if (y == imageWidth*1) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * 0.5;
        } else if (y == imageWidth*0) {
          VL53_weighted_matrix[y/imageWidth][x] = VL53_raw_matrix[y/imageWidth][x] * 1;
        } 

        //Serial.print(VL53_raw_matrix[y/imageWidth][x]);    
        }
        //Serial.println();
      }
      //Serial.println();
    }   
 }

 
  //Sum weighted and raw matrices
  weighted_sum = 0;
  raw_sum = 0;
  for (int row = 0; row <7; row++) {
    for (int col = 0; col < 5; col++) {
      weighted_sum += VL53_weighted_matrix[row][col];
      raw_sum += VL53_raw_matrix[row][col];
    }
  }

  //Use raw sums to distinguish between weights and poles/ramps
  if (raw_sum < 0) {
    //Then something is in the FoV
    if (raw_sum < -250) {
      pole_ramp_found = true; 
      Serial.println("pole ramp found");
    } else {
      weight_found = true; 
      Serial.println("weight found");
    }
  }
  
  //Use residual value to determine if the object is in the middle, left or right
  residual = weighted_sum/raw_sum; 
  if (weight_found) {
    if ((residual > 0.5) && (residual < 1.5)) {
      weight_right = true;
    } else if ((residual < -0.5) && (residual > -1.5)) {
      weight_left = true;
    } else  if ((residual > -0.5) && (residual < 0.5)){
      weight_middle = true;
    }
  }

  if (pole_ramp_found) {
    if ((residual > 0.5) && (residual < 1.5)) {
      pole_ramp_right = true;
    } else if ((residual < -0.5) && (residual > -1.5)) {
      pole_ramp_left = true;
    } else if ((residual > -0.5) && (residual < 0.5)){
      pole_ramp_middle = true;
    }
  }
}

void my_imagerintit() {
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
    
   if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  myImager.setResolution(8 * 8); //Enable all 64 pads

  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS); //Change to continuous to get data in constantly
  myImager.setSharpenerPercent(100); //Set sharpener percentage to avoid edges of objects being in adjacent zones
  myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  myImager.setRangingFrequency(30);
  myImager.startRanging();
}
