//************************************
//         sensors.h     
//************************************
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <vl53l5cx_plugin_detection_thresholds.h>
#ifndef SENSORS_H_
#define SENSORS_H_

extern SX1509 io;
extern int Right_sensor;
extern int Left_sensor;
extern int low_right_sensor;
extern int Encoder_Right;
extern int Encoder_Left;
extern int encoder_pickup;
extern int limit_switch_inner;
extern boolean weight_found;

extern bool limit_switch_outer;

extern int buffer1[4][4][10];
extern int averageinput[4][4];
extern int head;
extern int tail;

extern bool pole_ramp_middle;
extern bool pole_ramp_left;
extern bool pole_ramp_right;
extern bool weight_left;
extern bool weight_right;
extern bool weight_middle;
extern int CentiA;
extern int CentiB;


// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
void read_infrared();

void read_colour(/* Parameters */);

void read_limit();

// Pass in data and average the lot
void sensor_average(/* Parameters */);

void circ_buffer_add(int input[4][4]);

void average_Buffer();
void my_imagerintit();

#endif /* SENSORS_H_ */
