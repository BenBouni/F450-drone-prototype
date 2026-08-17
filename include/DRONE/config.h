#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>  

#include "Radio.h"        // defines Emitor_receptor, update_data, failsafe
#include "PID.h"          // defines PID class
#include "Failsafe.h"     // defines failsafe class
#include "updater.h"      // defines update_data class
#include "data.h"

// here you can adjust the configurations for the drone :
// motor 
extern mixMotor monMix; // global mixer object, declared extern in motorLOGIC.h and defined in config.cpp to avoid multiple-definition errors
// PID parameters
extern PID PIDroll;
extern PID PIDpitch;
extern PID PIDyaw;


extern update_data data; // in ms, to be adjusted according to system requirements

// failsafe delays :
extern failsafe monFailsafe; // temporary/critical delays in ms for signal loss handling

// tasks frequencies :
extern const TickType_t frequency_controll; // 4 ms for the control loop
extern const TickType_t frequency_captor;    // 2 ms for the captor task
extern const TickType_t frequency_radio;  // 100 Hz for the radio task
extern const TickType_t frequency_failsafe; // 1000 Hz for the failsafe task

#endif 