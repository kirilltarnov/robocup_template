//************************************
//         sensors.h     
//************************************
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <vl53l5cx_plugin_detection_thresholds.h>
#ifndef SENSORS_H_
#define SENSORS_H_

#define encoder1PinA 2
#define encoder1PinB 3
#define encoder2PinA 4
#define encoder2PinB 5


typedef struct {
    uint8_t sendPin;
    uint8_t receivePin;
    bool pulseSent;

    unsigned int startTicks;
    unsigned int endTicks;
    unsigned int value;
    unsigned int lastValidValue;
} ultrasonic_sensor;

void ultrasonic_ping(ultrasonic_sensor *sensor);
void ultrasonic_pong(ultrasonic_sensor *sensor);

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

extern bool pole_ramp_middle;
extern bool pole_ramp_left;
extern bool pole_ramp_right;
extern bool weight_left;
extern bool weight_right;
extern bool weight_middle;
extern int ultrasoundA;
extern int ultrasoundB;
extern int ultratimerA;
extern int ultratimerB;
extern int ultrathingA;
extern int ultrathingB;
extern int ultraA;
extern int ultraB;

extern ultrasonic_sensor leftUltrasonic;
extern ultrasonic_sensor rightUltrasonic;
extern bool US_left_wall_too_close;
extern bool US_right_wall_too_close;

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);
void ultrasonic_left_bool(ultrasonic_sensor *sensor);
void ultrasonic_left_bool(ultrasonic_sensor *sensor);

// Read infrared value
void read_infrared();

void read_colour(/* Parameters */);

void read_limit();

// Pass in data and average the lot
void sensor_average(/* Parameters */);

void circ_buffer_add(int input[4][4]);

void SENSOR_TOF();
void my_imagerinit();

#endif /* SENSORS_H_ */
