#ifndef RADIO_H
#define RADIO_H
#include <RF24.h>
#include "data.h"
#include "motorLOGIC.h"


class Emitor_receptor {
  public:
    // role can be changed at runtime if necessary
    enum Role { TRANSMITTER, RECEIVER };

  private:
    RF24 radio;
    uint8_t txAddress[6];               // pipe used when transmitting
    uint8_t rxAddress[6];               // pipe used when listening
    Role role;                          // current operating mode
    const unsigned long interval = 100; // sending interval in ms
    unsigned long dernierEnvoi;

    void configurePipes();              // helper used when role changes

  public:
    /**
     * @param cePin, csnPin    SPI pins for the nRF24L01
     * @param txAddr           address to use when sending
     * @param rxAddr           address to use when receiving
     * @param initialRole      start up as transmitter or receiver
     */
    Emitor_receptor(uint8_t cePin,
                    uint8_t csnPin,
                    const uint8_t txAddr[6],
                    const uint8_t rxAddr[6],
                    Role initialRole = RECEIVER);

    void begin();

    /**
     * Switch the module to the other role on the fly.
     * Calling this will re‑configure the pipes and start/stop listening
     * as appropriate. Useful if you want the same board to act as an
     * RC transmitter one moment and then listen for telemetry later.
     */
    void switchRole(Role newRole);

    bool receivePacket(ControlData& Packet);   // valid only when in RECEIVER mode
    bool sendPacket(const DroneData& packetDrone); // valid only in TRANSMITTER mode
    void sendDroneData(const DroneData& packetDrone); // helper for periodic sends
};

extern Emitor_receptor radio;

class failsafe {
  private:
    unsigned long lastSignalTime;
    const unsigned long timeoutmin; // in milliseconds
    const unsigned long timeoutmax ; // in milliseconds
   
  public:
    failsafe(unsigned long timeoutmin, unsigned long timeoutmax);
    void updateSignalTime();
    bool temporaryLoss();
    bool criticalLoss();
    void failsafeAction(mixMotor& mixer);
};

extern failsafe monFailsafe; // temporary delay / critical delay in ms, to be adjusted according to the expected signal loss duration and the desired failsafe behavior of the drone

class update_data {
  private :
  std::atomic <unsigned long> dernier_update {0};
  const unsigned long delai_update;
  std::atomic <bool> updated{false};
  public :
  update_data(unsigned long delaiupdate);
  void data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED);
  bool UPDATED() const;
};

extern update_data data;

#endif 

