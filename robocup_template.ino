
/********************************************************************************
                                 ROBOCUP TEMPLATE


    This is a template program design with modules for
    different components of the robot, and a task scheduler
    for controlling how frequently tasks sholud run


    written by: Logan Chatfield, Ben Fortune, Lachlan McKenzie, Jake Campbell

 ******************************************************************************/

//control the DC motors

//#include <Herkulex.h>             //smart servo
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler 
#include <stdlib.h>                  //contains random number generator 
#include <SparkFunSX1509.h> // Include SX1509 library
#include <SparkFun_VL53L5CX_Library.h>
#include <vl53l5cx_plugin_detection_thresholds.h>
#include <eloquent.h>

// Custom headers
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h"


//**********************************************************************************
// Local Definitions
//**********************************************************************************
// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

// SX1509 Pins:
const byte SX1509_AIO0 = 0;
const byte SX1509_AIO1 = 1;
const byte SX1509_AIO2 = 2;
const byte SX1509_AIO3 = 3;
const byte SX1509_AIO4 = 4;
const byte SX1509_AIO5 = 5;
const byte SX1509_AIO6 = 6;
const byte SX1509_AIO7 = 7;
const byte SX1509_AIO8 = 8;
const byte SX1509_AIO9 = 9;
const byte SX1509_AIO10 = 10;
const byte SX1509_AIO11 = 11;
const byte SX1509_AIO12 = 12;
const byte SX1509_AIO13 = 13;
const byte SX1509_AIO14 = 14;
const byte SX1509_AIO15 = 15;

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define US_READ_TASK_PERIOD                 40
#define IR_READ_TASK_PERIOD                 40
#define COLOUR_READ_TASK_PERIOD             40
#define SENSOR_AVERAGE_PERIOD               40
#define read_limit_TASK_PERIOD            0
#define SET_MOTOR_TASK_PERIOD               40
#define WEIGHT_SCAN_TASK_PERIOD             40
#define COLLECT_WEIGHT_TASK_PERIOD          40
#define RETURN_TO_BASE_TASK_PERIOD          40
#define DETECT_BASE_TASK_PERIOD             40
#define UNLOAD_WEIGHTS_TASK_PERIOD          40
#define CHECK_WATCHDOG_TASK_PERIOD          40
#define VICTORY_DANCE_TASK_PERIOD           40
#define Jamming_check_period                40             




// Task execution amount definitions
// -1 means indefinitely
#define US_READ_TASK_NUM_EXECUTE           0
#define IR_READ_TASK_NUM_EXECUTE           -1
#define COLOUR_READ_TASK_NUM_EXECUTE       0
#define read_limit_TASK_NUM_EXECUTE      0
#define SENSOR_AVERAGE_NUM_EXECUTE         0
#define SET_MOTOR_TASK_NUM_EXECUTE         -1
#define WEIGHT_SCAN_TASK_NUM_EXECUTE       -1
#define COLLECT_WEIGHT_TASK_NUM_EXECUTE    0
#define RETURN_TO_BASE_TASK_NUM_EXECUTE    0
#define DETECT_BASE_TASK_NUM_EXECUTE       0
#define UNLOAD_WEIGHTS_TASK_NUM_EXECUTE    0
#define CHECK_WATCHDOG_TASK_NUM_EXECUTE    0
#define VICTORY_DANCE_TASK_NUM_EXECUTE     0
#define Jamming_check_NUM_EXECUTE          1   

// Pin definitions
#define IO_POWER  49
#define encoder1PinA 2
#define encoder1PinB 3
#define encoder2PinA 4
#define encoder2PinB 5
#define encoder3PinA 31 //Pick-up motor encoder
#define encoder3PinB 30
#define encoder1serialpin 0
#define encoder2serialpin 1
#define encoder3serialpin 7
#define limit_switch_pin 20
#define JOYSTICK_PIN 27

//VL53L5CX object detection set-up
#define WEIGHT_ZONE_NUM 1 //a weight should take up a certain amount of zones
#define VL53L5CX_MEDIAN_RANGE_MM 200 //range to detect weight from [mm]
#define DISTANCE_CHANGE 200 //used to detect a change of distance in a zone (weight detection)


// Serial deffinitions
#define BAUD_RATE 9600

int Encoder_Left = 0;
int Encoder_Right = 0;
int encoder_pickup = 0;

int joystick_x_pos = 0;
int joystick_map_x = 0;
boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;
boolean A_set3 = false;
boolean B_set3 = false;
boolean weight_found = false;
boolean set_thresh_enable = true;
boolean get_thresh_enable = true;
boolean set_thresh_status = true;
boolean get_thresh_status = true;
uint16_t measurement_old [3][3]; //16 int array to hold previous time-step data

Servo right_motor;
Servo left_motor;
Servo Gate_servo;
Servo pickup_motor;
int State = 0;

int Right_sensor;
int Left_sensor;
void infra_red_callback();

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
VL53L5CX_DetectionThresholds detectionThresholds;

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

double measurement_rounded = 0;

// data variables

//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */


// Tasks for reading sensors
Task tRead_ultrasonic(US_READ_TASK_PERIOD,       US_READ_TASK_NUM_EXECUTE,        &read_ultrasonic);
Task tRead_infrared(IR_READ_TASK_PERIOD,         IR_READ_TASK_NUM_EXECUTE,        &read_infrared);
Task tRead_colour(COLOUR_READ_TASK_PERIOD,       COLOUR_READ_TASK_NUM_EXECUTE,    &read_colour);
Task tread_limit(read_limit_TASK_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &read_limit);
Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);
//Task tJammingcheck(Jamming_check_period,         Jamming_check_NUM_EXECUTE,       &Jamming);
// Task to set the motor speeds and direction
Task tSet_motor(SET_MOTOR_TASK_PERIOD,           SET_MOTOR_TASK_NUM_EXECUTE,      &DC_motors);

// Tasks to scan for weights and collection upon detection
Task tWeight_scan(WEIGHT_SCAN_TASK_PERIOD,       WEIGHT_SCAN_TASK_NUM_EXECUTE,    &State_machine);
Task tCollect_weight(COLLECT_WEIGHT_TASK_PERIOD, COLLECT_WEIGHT_TASK_NUM_EXECUTE, &collect_weight);

// Tasks to search for bases and unload weights
Task tReturn_to_base(RETURN_TO_BASE_TASK_PERIOD, RETURN_TO_BASE_TASK_NUM_EXECUTE, &return_to_base);
Task tDetect_base(DETECT_BASE_TASK_PERIOD,       DETECT_BASE_TASK_NUM_EXECUTE,    &detect_base);
Task tUnload_weights(UNLOAD_WEIGHTS_TASK_PERIOD, UNLOAD_WEIGHTS_TASK_NUM_EXECUTE, &unload_weights);

// Tasks to check the 'watchdog' timer (These will need to be added in)
//Task tCheck_watchdog(CHECK_WATCHDOG_TASK_PERIOD, CHECK_WATCHDOG_TASK_NUM_EXECUTE, &check_watchdog);
//Task tVictory_dance(VICTORY_DANCE_TASK_PERIOD,   VICTORY_DANCE_TASK_NUM_EXECUTE,  &victory_dance);

Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  pin_init();
  robot_init();
  task_init();

}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work)
// Set as high or low
//**********************************************************************************
void pin_init() {

  Serial.println("Pins have been initialised \n");

  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
  pinMode(encoder1PinA, INPUT);       //Set encoder pins as inputs
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);
  pinMode(encoder3PinA, INPUT);
  pinMode(encoder3PinB, INPUT);
  pinMode(limit_switch_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  //Set up an interrupt for each encoder
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), doEncoder3A, CHANGE);
  right_motor.attach(encoder2serialpin);
  left_motor.attach(encoder1serialpin);
  //Gate_servo.attach(7);
  pickup_motor.attach(encoder3serialpin);

  if (!io.begin(SX1509_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }

  io.pinMode(SX1509_AIO0, INPUT);
  io.pinMode(SX1509_AIO1, INPUT);
  io.pinMode(SX1509_AIO2, INPUT);
  io.pinMode(SX1509_AIO3, INPUT);
  io.pinMode(SX1509_AIO4, INPUT);
  io.pinMode(SX1509_AIO5, INPUT);
  io.pinMode(SX1509_AIO6, INPUT);
  io.pinMode(SX1509_AIO7, INPUT);
  io.pinMode(SX1509_AIO8, INPUT);
  io.pinMode(SX1509_AIO9, INPUT);
  io.pinMode(SX1509_AIO10, INPUT);
  io.pinMode(SX1509_AIO11, INPUT);
  io.pinMode(SX1509_AIO12, INPUT);
  io.pinMode(SX1509_AIO13, INPUT);
  io.pinMode(SX1509_AIO14, INPUT);
  io.pinMode(SX1509_AIO15, INPUT);

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  myImager.setResolution(4 * 4); //Enable all 64 pads

  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS); //Change to continuous to get data in constantly
  myImager.setSharpenerPercent(100); //Set sharpener percentage to avoid edges of objects being in adjacent zones
  myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  myImager.startRanging();

//  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
//  if (myImager.begin() == false)
//  {
//    Serial.println(F("Sensor not found - check your wiring. Freezing"));
//    while (1) ;
//  }


  // //enable detection thresholds, '1' to enable thresholds. This allows to set parameters and get parameters
  // set_thresh_enable = vl53l5cx_set_detection_thresholds_enable(myimager.dev, 1);
  // get_thresh_enable = vl53l5cx_get_detection_thresholds_enable(myimager.dev, 1); 
 
  // //parametise the detection thresholds
  // detectionThresholds.zone_num = WEIGHT_ZONE_NUM;
  // detectionThresholds.measurement = VL53L5CX_MEDIAN_RANGE_MM;
  // detectionThresholds.type = VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER; //**NOT 100% sure on which type makes sense

  // //set thresholds
  // set_thresh_status = vl53l5cx_set_detection_thresholds(myimager.dev, &detectionThresholds);

  // //get_thresh_status = vl53l5cx_get_detection_thresholds(myimager.dev, &detectionThresholds); 
  
  

}

//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
  Serial.println("Robot is ready \n");
}

//**********************************************************************************
// Initialise the tasks for the scheduler
//**********************************************************************************
void task_init() {

  // This is a class/library function. Initialise the task scheduler
  taskManager.init();

  // Add tasks to the scheduler
  taskManager.addTask(tRead_ultrasonic);   //reading ultrasonic
  taskManager.addTask(tRead_infrared);
  taskManager.addTask(tRead_colour);
  taskManager.addTask(tSensor_average);
  taskManager.addTask(tread_limit);
  taskManager.addTask(tSet_motor);
  taskManager.addTask(tWeight_scan);
  taskManager.addTask(tCollect_weight);
  taskManager.addTask(tReturn_to_base);
  taskManager.addTask(tDetect_base);
  taskManager.addTask(tUnload_weights);
  //taskManager.addTask(tJammingcheck);

  //taskManager.addTask(tCheck_watchdog);
  //taskManager.addTask(tVictory_dance);

  //enable the tasks
  //  tRead_ultrasonic.enable();
  // tRead_infrared.enable();
  // tread_encoder.enable();
  //  tRead_colour.enable();
  //  tSensor_average.enable();
  // tread_limit.enable();
  // tSet_motor.enable();
  // tWeight_scan.enable();
  //  tCollect_weight.enable();
  //  tReturn_to_base.enable();
  //  tDetect_base.enable();
  //  tUnload_weights.enable();
  //tCheck_watchdog.enable();
  //tVictory_dance.enable();

  Serial.println("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  taskManager.execute();    //execute the scheduler
  //Serial.println(joystick_map_x);
  //Joystick for testing only
  joystick_x_pos = analogRead(JOYSTICK_PIN);
  joystick_map_x = map(joystick_x_pos, 0, 950, 1050, 1950);
  // if (set_thresh_enable == 0) {
  //   Serial.println("VL sensor threshold ENABLED");
  // }
  // if (set_thresh_status == 0) {
  //   Serial.println("VL sensor threshold SET");
  // }
  // weight_found = io.digitalRead(SX1509_AIO0); 
  // Serial.println(weight_found);
 //Poll sensor for new data (ToF)
  
 if (myImager.isDataReady() == true)
 {
   if (myImager.getRangingData(&measurementData)) //Read distance data into array
   {
      
     //The ST library returns the data transposed from zone mapping shown in datasheet
     //Pretty-print data with increasing y, decreasing x to reflect reality
     for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
     {
       for (int x = imageWidth - 1 ; x >= 0 ; x--)
       {
         Serial.print("\t");
         measurement_rounded = measurementData.distance_mm[x+y]/10;
         measurement_rounded = round(measurement_rounded)*10;
         measurement_rounded = int(measurement_rounded); //convert from double to int
         measurement_old[y][x] = measurement_rounded; //place rounded data in a 
        //  Serial.print(measurement_old);
        //  if (abs(measurement_rounded-measurement_old) > DISTANCE_CHANGE) {
        //     Serial.print("Weight Found");
        //  }
        
       }
       Serial.println();
     }
     Serial.println();
   }
 }
 delay(5); //Small delay between polling
 

//  if (pickup_mechanism == WEIGHT_FOUND) {
//    tJammingcheck.enable();
//  } else{
//    tJammingcheck.disable();
//  }
  //Poll sensor for new data (ToF)
//  if (myImager.isDataReady() == true)
//  {
//    if (myImager.getRangingData(&measurementData)) //Read distance data into array
//    {
//      //The ST library returns the data transposed from zone mapping shown in datasheet
//      //Pretty-print data with increasing y, decreasing x to reflect reality
//      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
//      {
//        for (int x = imageWidth - 1 ; x >= 0 ; x--)
//        {
//          Serial.print("\t");
//          Serial.print(measurementData.distance_mm[x + y]);
//        }
//        Serial.println();
//      }
//      Serial.println();
//    }
//  }
//  delay(5); //Small delay between polling
  //limit_switch = digitalRead(limit_switch_pin) == HIGH;
//  Serial.println(limit_switch);
//  Serial.println(State);
//  Serial.println("Another scheduler execution cycle has oocured \n");
}

void doEncoder1A() {
  // Test transition
  A_set1 = digitalRead(encoder1PinA) == HIGH;
  // and adjust counter + if A leads B
  Encoder_Right += (A_set1 != B_set1) ? +1 : -1;

  B_set1 = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  Encoder_Right += (A_set1 == B_set1) ? +1 : -1;
}


// Interrupt on A changing state
void doEncoder2A() {
  // Test transition
  A_set2 = digitalRead(encoder2PinA) == HIGH;
  // and adjust counter + if A leads B
  Encoder_Left -= (A_set2 != B_set2) ? +1 : -1;

  B_set2 = digitalRead(encoder2PinB) == HIGH;
  // and adjust counter + if B follows A
  Encoder_Left -= (A_set2 == B_set2) ? +1 : -1;
}

void doEncoder3A() {
  // Test transition
  A_set3 = digitalRead(encoder3PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder_pickup += (A_set3 != B_set3) ? +1 : -1;

  B_set3 = digitalRead(encoder3PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder_pickup += (A_set3 == B_set3) ? +1 : -1;
}
