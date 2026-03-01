#include "config.h"

// global objects defined in one translation unit to avoid multiple-definition errors
PID PIDroll(1.0, 0.0, 0.0);
PID PIDpitch(1.0, 0.0, 0.0);
PID PIDyaw(1.0, 0.0, 0.0);

Emitor_receptor radio(CE_PIN, CSN_PIN);
update_data data(500);
failsafe monFailsafe(1000, 10000);

const TickType_t frequency_controll = 4 / portTICK_PERIOD_MS;
const TickType_t frequency_captor    = 2 / portTICK_PERIOD_MS;
const TickType_t frequency_radio    = 10 / portTICK_PERIOD_MS;

// mutex handles and shared-data objects declared in data.h
SemaphoreHandle_t xMutexRadio = NULL;
SemaphoreHandle_t xMutexControll = NULL;
SemaphoreHandle_t xMutexTele = NULL;
SemaphoreHandle_t xMutexFailsafe = NULL;

Order order;
telemerie tele;
DroneData dronepacket;
ControlData Packet;
