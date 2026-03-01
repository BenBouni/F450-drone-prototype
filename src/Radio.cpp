#include "Radio.h"
#include "data.h"
#include "motorLOGIC.h"

// Emitor_receptor implementation

Emitor_receptor::Emitor_receptor(uint8_t cePin, uint8_t csnPin) : radio(cePin, csnPin) {}

void Emitor_receptor::begin() {
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.startListening();
    dernierEnvoi = millis();
}

bool Emitor_receptor::receivePacket(ControlData& Packet) {
    if (radio.available()) {
        radio.read(&Packet, sizeof(Packet));
        return true;
    } else return false;        
}

bool Emitor_receptor::sendPacket(const DroneData& packetDrone) {
    dernierEnvoi = millis();
    radio.stopListening();
    bool success = radio.write(&packetDrone, sizeof(packetDrone));
    radio.startListening();
    return success;
}

void Emitor_receptor::sendDroneData(const DroneData& packetDrone) {
    unsigned long currentMillis = millis();
    if (currentMillis - dernierEnvoi >= interval) {
        sendPacket(packetDrone);
        dernierEnvoi = currentMillis;
    }
} 

// failsafe implementation 

    failsafe::failsafe(unsigned long timeoutmin, unsigned long timeoutmaxs) : timeoutmin(timeoutmin), timeoutmax(timeoutmaxs) {
      lastSignalTime = millis();
    }

    void failsafe::updateSignalTime() { // to be called when a valid signal is received
        lastSignalTime = millis();
    }

    bool failsafe::temporaryLoss() {
      if (millis() - lastSignalTime > timeoutmin)
      return true;
      return false;
    } 
    bool failsafe::criticalLoss() {
      if (millis() - lastSignalTime > timeoutmax)
      return true;
      return false;
    }
    void failsafe::failsafeAction(mixMotor& mixer) {
      if (criticalLoss() ) {
        mixer.stopTout();
      } else if (temporaryLoss() ) {
        if (xSemaphoreTake(xMutexControll, 0) == pdTRUE){
          order.thr = 1300.0f; // reduce throttle to a low value to allow for a safe landing if the signal comes back, but cut the PID corrections to avoid any sudden movements of the drone in case of abberant mesures or code blocking, and to allow for a safe landing if the signal comes back.
          order.pitch = 0.0f;
          order.roll = 0.0f;
          order.yaw = 0.0f;
          order.armed = false; // Disarm the drone
          xSemaphoreGive(xMutexControll);
        }; // stabilisation incase of temporary loss of signal, to avoid a crash while waiting for the signal to come back, by cutting the throttle and disarming the drone, but keeping the PID corrections to 0 to avoid any sudden movements of the drone in case of abberant mesures or code blocking, and to allow for a safe landing if the signal comes back.
    } else {
        // normal operation, do nothing
      }
    }

// data update implementation

  update_data::update_data(unsigned long delaiupdate) : delai_update(delaiupdate)  {} // delaiupdate is the maximum allowed time between two valid data reception from the GS

  void update_data::data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED) {
   ControlData Packet;
   if (radio.receivePacket(Packet)) {
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

  