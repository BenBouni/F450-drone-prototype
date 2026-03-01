#ifndef CONFIG_H
#define CONFIG_H
#include "Pins.h"
#include "IMU.h"
#include "Radio.h"
#include "motorLOGIC.h"

// here you can adjust the configurations for the drone :

// PID constants :
extern PID PIDroll;
extern PID PIDpitch;
extern PID PIDyaw;

//radio pins :
extern Emitor_receptor radio;
//data check rate :
extern update_data data; // in ms, to be adjusted according to the needs of the

//failsafe delays :
extern failsafe monFailsafe; // temporary delay / critical delay in ms, to be adjusted according to the expected signal loss duration and the desired failsafe behavior of the drone

//tasks frequencies :
extern const TickType_t frequency_controll; // 4 ms for the control loop,  because it is a good compromise between the performance of the PID loop and the performance of the ESP32
extern const TickType_t frequency_captor;    // 2 ms for the captor task, because it is enough for the PID/controll loop
extern const TickType_t frequency_radio;  // 100 Hz for the radio task , because it is enough for the radio communication and to avoid overloading the CPU with too many tasks running at high frequency

#endif 