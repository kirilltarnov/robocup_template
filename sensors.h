//************************************
//         sensors.h     
//************************************
#include <Wire.h>
#include <SparkFunSX1509.h>
#ifndef SENSORS_H_
#define SENSORS_H_


extern SX1509 io;
extern int Right_sensor;
extern int Left_sensor;
extern int low_right_sensor;
extern int Encoder_Right;
extern int Encoder_Left;
extern int encoder_pickup;
extern int limit_switch;
extern int buffer1[4][4][10];
extern int averageinput[4][4];
extern int head;
extern int tail;



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

#endif /* SENSORS_H_ */
