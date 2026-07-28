
#include "updater.h"
#include "Radio.h"
#include "Failsafe.h"
#include "data.h"
  
  update_data::update_data(unsigned long delaiupdate) : delai_update(delaiupdate)  {} // delaiupdate is the maximum allowed time between two valid data reception from the GS

  void update_data::data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED) {
    // Receive control packet from GCS (ControlData) and store into atomics safely
    ControlData receivedPacket;
    if (radio.receivePacket(receivedPacket)) {
      // store atomically so other tasks can read consistently without taking mutexes
      THR.store(receivedPacket.thr);
      PITCH.store(receivedPacket.pitch);
      ROLL.store(receivedPacket.roll);
      YAW.store(receivedPacket.yaw);
      ARMED.store(receivedPacket.armed);
      if (xSemaphoreTake(xMutexFailsafe, 0) == pdTRUE) {
        monFailsafe.updateSignalTime();
        xSemaphoreGive(xMutexFailsafe);
      }
      dernier_update.store(millis());
    }
    // updated flag based on last reception
    if (millis() - dernier_update.load() > delai_update) {
      updated.store(false);
    } else {
      updated.store(true);
    }
  }
  bool update_data::UPDATED() const { return updated.load(); }