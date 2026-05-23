
#include "updater.h"
#include "Radio.h"
#include "Failsafe.h"
#include "data.h"
  
  update_data::update_data(unsigned long delaiupdate) : delai_update(delaiupdate)  {} // delaiupdate is the maximum allowed time between two valid data reception from the GS

  void update_data::data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED) {
   ControlData Packet;
   if (radio.receivePacket(dronepacket)) {
     THR = Packet.thr;
     PITCH = Packet.pitch;
     ROLL = Packet.roll;
     YAW = Packet.yaw;
     ARMED = Packet.armed;
    if (xSemaphoreTake(xMutexFailsafe, 0) == pdTRUE) {
      monFailsafe.updateSignalTime();
      xSemaphoreGive(xMutexFailsafe);
    }
    dernier_update = millis();
   } 
   if (millis()-dernier_update > delai_update) {
    updated = false;
   } else updated = true; 
  }
  bool update_data::UPDATED() const { return updated.load(); }