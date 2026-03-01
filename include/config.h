#ifndef CONFIG_H
#define CONFIG_H

#include "Pins.h"
#include <Arduino.h>  // provides TickType_t and commonly used types

// bring in definitions used by the extern declarations
#include "motorLOGIC.h"   // defines PID, mixMotor
#include "Radio.h"        // defines Emitor_receptor, update_data, failsafe

// here you can adjust the configurations for the drone :

// PID constants :
extern PID PIDroll;
extern PID PIDpitch;
extern PID PIDyaw;

// radio and data objects used by the tasks
extern Emitor_receptor radio;
extern update_data data; // in ms, to be adjusted according to system requirements

// failsafe delays :
extern failsafe monFailsafe; // temporary/critical delays in ms for signal loss handling

// tasks frequencies :
extern const TickType_t frequency_controll; // 4 ms for the control loop
extern const TickType_t frequency_captor;    // 2 ms for the captor task
extern const TickType_t frequency_radio;  // 100 Hz for the radio task
extern const TickType_t frequency_failsafe; // 1000 Hz for the failsafe task

#endif 