//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

extern int Right_sensor;
extern int Left_sensor;
extern int Encoder_Right;
extern int Encoder_Left;

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
void read_infrared();

void read_colour(/* Parameters */);

// Pass in data and average the lot
void sensor_average(/* Parameters */);

#endif /* SENSORS_H_ */
