#include "config.h"
#include "motorLOGIC.h"  // PID and mixMotor definitions
#include "Radio.h"       // Emitor_receptor, update_data, failsafe

// global objects defined in one translation unit to avoid multiple-definition errors
PID PIDroll(1.0, 0.0, 0.0);
PID PIDpitch(1.0, 0.0, 0.0);
PID PIDyaw(1.0, 0.0, 0.0);

uint8_t txAddress[6] = "00001";
uint8_t rxAddress[6] = "00002";

Emitor_receptor radio(100, CE_PIN, CSN_PIN, txAddress, rxAddress); // instantiate radio object with defined pins and addresses
update_data data(500);
failsafe monFailsafe(1000, 10000);

// instantiate mixer (declared extern in motorLOGIC.h)
mixMotor monMix;

const TickType_t frequency_controll = pdMS_TO_TICKS(4);
const TickType_t frequency_captor    = pdMS_TO_TICKS(2);
const TickType_t frequency_radio    = pdMS_TO_TICKS(10);
const TickType_t frequency_failsafe  = pdMS_TO_TICKS(100);

// mutex handles and shared-data objects declared in data.h
SemaphoreHandle_t xMutexRadio = NULL;
SemaphoreHandle_t xMutexControll = NULL;
SemaphoreHandle_t xMutexTele = NULL;
SemaphoreHandle_t xMutexFailsafe = NULL;

Order order;
telemerie tele;
DroneData dronepacket;
ControlData Packet;
