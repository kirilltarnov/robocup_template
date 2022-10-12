
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

// Custom headers
#include "motors.h"
#include "sensors.h"


//**********************************************************************************
// Local Definitions
//**********************************************************************************
// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

// SX1509 Pins:

const byte SX1509_AIO15 = 15;

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define US_READ_TASK_PERIOD                 200
#define IR_READ_TASK_PERIOD                 10
#define COLOUR_READ_TASK_PERIOD             40
#define SENSOR_TOF_PERIOD                   40
#define read_limit_TASK_PERIOD              60
#define SET_NAVIGATION_TASK_PERIOD     20
#define SET_PICKUP_TASK_PERIOD         20

        




// Task execution amount definitions
// -1 means indefinitely
#define US_READ_TASK_NUM_EXECUTE           -1
#define IR_READ_TASK_NUM_EXECUTE           -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1
#define read_limit_TASK_NUM_EXECUTE        -1
#define SENSOR_TOF_NUM_EXECUTE             -1
#define SET_NAVIGATION_TASK_NUM_EXECUTE         -1
#define SET_PICKUP_TASK_NUM_EXECUTE        -1


// Pin definitions
#define IO_POWER  49
#define encoder3PinA 31 //Pick-up motor encoder
#define encoder3PinB 30
#define encoder1serialpin 0
#define encoder2serialpin 1
#define encoder3serialpin 7
#define limit_switch_pin 20
#define limit_switch2_pin 14
#define JOYSTICK_PIN 27


//VL53L5CX object detection set-up
#define WEIGHT_ZONE_NUM 1 //a weight should take up a certain amount of zones
#define VL53L5CX_MEDIAN_RANGE_MM 200 //range to detect weight from [mm]
#define DISTANCE_CHANGE 200 //used to detect a change of distance in a zone (weight detection)
#define WEIGHT_DISTANCE_ZONE -150
#define POLE_DISTANCE_ZONE -250


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
boolean set_thresh_enable = true;
boolean get_thresh_enable = true;
boolean set_thresh_status = true;
boolean get_thresh_status = true;

// uint16_t col_sum[6];
// uint16_t col_sum_old[6];
// uint16_t row_sum[8];
// uint16_t row_sum_old[8];
// int8_t weight_row_zone = 0;
// int8_t weight_col_zone = 0;
int16_t row_difference = 0; 
int16_t col_difference = 0; 



Servo right_motor;
Servo left_motor;
Servo Gate_servo;
Servo pickup_motor;
int State = 0;

int Right_sensor;
int Left_sensor;
void infra_red_callback();

VL53L5CX_DetectionThresholds detectionThresholds;


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
Task tread_limit(read_limit_TASK_PERIOD,      read_limit_TASK_NUM_EXECUTE,      &read_limit);
Task tSENSOR_TOF(SENSOR_TOF_PERIOD,      SENSOR_TOF_NUM_EXECUTE,      &SENSOR_TOF);
Task tnavigation(SET_NAVIGATION_TASK_PERIOD,           SET_NAVIGATION_TASK_NUM_EXECUTE,      &navigation);
Task tpickup(SET_PICKUP_TASK_PERIOD,           SET_PICKUP_TASK_NUM_EXECUTE,      &pickup);

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

    // Set up the timer source
    // Bits 8 - 6 indicate to use the 24MHz internal high speed clock
    // Bit 0 enables the counter
    GPT1_CR = (GPT1_CR & (~(1 << 6 | 1 << 8))) | (1 << 7) | (1 << 0); 

    // Set the prescaler to 35 so that 1mm = 1 tick
    GPT1_PR = 140; 

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
  pinMode(32, OUTPUT);
  pinMode(33, INPUT);
  pinMode(30, OUTPUT);
  pinMode(31,INPUT);

  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  

  pinMode(limit_switch2_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  //Set up an interrupt for each encoder
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(leftUltrasonic.receivePin), processUltrasoundLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightUltrasonic.receivePin), processUltrasoundRight, CHANGE);
  
  right_motor.attach(encoder2serialpin);
  left_motor.attach(encoder1serialpin);
  //Gate_servo.attach(7);
  pickup_motor.attach(encoder3serialpin);

  if (!io.begin(SX1509_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }


  io.pinMode(SX1509_AIO15, INPUT);

  my_imagerintit();
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
  taskManager.addTask(tSENSOR_TOF);
  taskManager.addTask(tread_limit);
  taskManager.addTask(tnavigation);
  taskManager.addTask(tpickup);
  
  //enable the tasks
    tRead_ultrasonic.enable();
   tRead_infrared.enable();
    tRead_colour.enable();
    tSENSOR_TOF.enable();
   tread_limit.enable();
   tnavigation.enable();
   tpickup.enable();

  Serial.println("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  taskManager.execute();    //execute the scheduler
  //Joystick for testing only
  joystick_x_pos = analogRead(JOYSTICK_PIN);
  joystick_map_x = map(joystick_x_pos, 0, 950, 1050, 1950);
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


void processUltrasoundLeft() {
  ultrasonic_pong(&leftUltrasonic);
}

void processUltrasoundRight() {
  ultrasonic_pong(&rightUltrasonic);
}
