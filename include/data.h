#ifndef DATA_H
#define DATA_H
#include <Arduino.h>
#include <atomic>

struct ControlData {
  float thr; //1000 à 1600
  float yaw; // 0 à 100
  float pitch;
  float roll;     
  bool armed;      
};

struct DroneData { 
  float batteryVoltage;
  float ActualPitch;
  float ActualRoll;
  float ActualYaw;
};
 
struct Order {
  std::atomic <float> thr;
  std::atomic <float> pitch;
  std::atomic <float> roll;
  std::atomic <float> yaw;
  std::atomic <bool> armed;
};

struct telemerie {
    std::atomic <float> batteryVoltage;
    std::atomic <float> ActualPitch;
    std::atomic <float> ActualRoll;
    std::atomic <float> ActualYaw;
    };

extern SemaphoreHandle_t xMutexRadio; // handle received data
extern SemaphoreHandle_t xMutexControll; // handell data in the PID loop
extern SemaphoreHandle_t xMutexTele; // handle telemetry data to send to GS
extern SemaphoreHandle_t xMutexFailsafe; // handle failsafe conditions

extern Order order; // global order object to be shared between the radio task and the PID/controll task, to store the latest received control data from the GS
extern telemerie tele; // global telemetry object to be shared between the PID/controll task and the radio task, to store the latest calculated telemetry data to send to the GS
extern DroneData dronepacket; // global drone data object to be shared between the PID/controll task and the radio task, to store the latest calculated drone data to send to the GS
extern ControlData Packet; // global control data object to be shared between the radio task and the PID/controll task, to store the latest received control data from the GS



#endif