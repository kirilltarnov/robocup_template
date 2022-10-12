//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.


#include "sensors.h"
#include "Arduino.h"

// Local definitions



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
bool US_left_wall_too_close = false;
bool US_right_wall_too_close = false;

const unsigned int ULTRASONIC_UNKNOWN = __UINT32_MAX__;

ultrasonic_sensor rightUltrasonic = ultrasonic_sensor {.sendPin = 32, .receivePin = 33, .pulseSent = false };
ultrasonic_sensor leftUltrasonic = ultrasonic_sensor {.sendPin = 30, .receivePin = 31, .pulseSent = false };

void ultrasonic_ping(ultrasonic_sensor *sensor) {
  sensor->pulseSent = false;
  digitalWriteFast(sensor->sendPin, HIGH);
  sensor->startTicks = GPT1_CNT;
  sensor->value = ULTRASONIC_UNKNOWN;
  delayMicroseconds(10);
  digitalWriteFast(sensor->sendPin, LOW);
  sensor->pulseSent = true;
}

void ultrasonic_pong(ultrasonic_sensor *sensor) {
  if (!digitalReadFast(sensor->receivePin) && sensor->pulseSent) {
    sensor->endTicks = GPT1_CNT;
    sensor->value = sensor->endTicks - sensor->startTicks;
    sensor->lastValidValue = sensor->value;
  }
}

void ultrasonic_left_bool(ultrasonic_sensor *sensor) {
  if (sensor->lastValidValue < 200) {     
    US_left_wall_too_close = true;
  } else {
    US_left_wall_too_close = false; 
  }
}

void  ultrasonic_right_bool(ultrasonic_sensor *sensor) {
  if (sensor->lastValidValue < 200) {
    US_right_wall_too_close = true;
  } else {
    US_right_wall_too_close = false; 
  }
}


// Read ultrasonic value
void read_ultrasonic(/* Parameters */){
  //ultrasonic_print(&leftUltrasonic);
  //ultrasonic_print(&rightUltrasonic);

  ultrasonic_ping(&leftUltrasonic);
  ultrasonic_ping(&rightUltrasonic);

  ultrasonic_left_bool(&leftUltrasonic);
  ultrasonic_right_bool(&rightUltrasonic);

  if (US_left_wall_too_close == true) {
    //Serial.print("Ultrasonic left too close");
  } else if (US_right_wall_too_close == true) {
    //Serial.print("Ultrasonic right too close");
  }
}

void ultrasonic_print(ultrasonic_sensor *sensor) {
  // Serial.printf("%d: Value: %u Last Value: %u\r\n", 
  //   sensor->sendPin, 
  //   sensor->value, 
  //   sensor->lastValidValue
  // );
  
}

// Read infrared value
void read_infrared(){
  Left_sensor = analogRead(A9);  
  Right_sensor = analogRead(A8); 
  low_right_sensor = analogRead(A7);  
  Serial.println(low_right_sensor);
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
  
}

void read_limit() {
    limit_switch_inner = digitalRead(20);
    limit_switch_outer = digitalRead(14);
}


// Pass in data and average the lot
void SENSOR_TOF(/* Parameters */){
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
        Serial.print("\t");
        // take the raw data from the VL sensor, round it to a number of 10
        //  measurement_rounded = measurementData.distance_mm[x+y]/10;
        //  measurement_rounded = round(measurement_rounded)*10;
        //  measurement_rounded = int(measurement_rounded); //convert from double to int
        VL53_raw_matrix[y/imageWidth][x] = measurementData.distance_mm[x+y]; //place rounded data in a matrix 
        if (x == 5) {
          VL53_raw_matrix[y/imageWidth][x] -= 520;
        } else if (x == 4) {
          VL53_raw_matrix[y/imageWidth][x] -= 420;
        } else if (x == 3) {
          VL53_raw_matrix[y/imageWidth][x] -= 360;
        } else if (x == 2) {
          VL53_raw_matrix[y/imageWidth][x] -= 310;
        } else if (x == 1) {
          VL53_raw_matrix[y/imageWidth][x] -= 270;
        } else if (x == 0) {
          VL53_raw_matrix[y/imageWidth][x] -= 255;
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

        Serial.print(VL53_raw_matrix[y/imageWidth][x]);    
        }
        Serial.println();
      }
      Serial.println();
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

  Serial.print("Raw sum: ");
  Serial.println(raw_sum);

  //Use raw sums to distinguish between weights and poles/ramps
  if (raw_sum < -100) {
    //Then something is in the FoV
    if (raw_sum < -2000) {
      pole_ramp_found = true; 
      //Serial.println("pole ramp found");
    } else if (raw_sum < 0) {
      weight_found = true; 
      //Serial.println("weight found");
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

void my_imagerinit() {
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
