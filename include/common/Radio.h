#ifndef RADIO_H
#define RADIO_H
#include <RF24.h>
#include "data.h"


template <typename sent, typename received>
class Emitor_receptor {
  private:

    RF24 radio;
    uint8_t txAddress[6];               // pipe used when transmitting
    uint8_t rxAddress[6];               // pipe used when listening
    int CE_PIN ;
    int CSN_PIN ;
    unsigned long interval; // sending interval in ms
        unsigned long dernierEnvoi;
    
    public:
        Emitor_receptor(const int interval, const int CE_PIN, const int CSN_PIN, const uint8_t tx[6], const uint8_t rx[6]) : 
                interval(interval), CE_PIN(CE_PIN), CSN_PIN(CSN_PIN), radio(CE_PIN, CSN_PIN) {
          memcpy(txAddress, tx, 6);
          memcpy(rxAddress, rx, 6);
      }

        void begin() {
            if (!radio.begin()) {
                Serial.println("Radio hardware not responding!");
                while (1) {} // halt if radio is not working
            }
            radio.openReadingPipe(1, rxAddress);
            radio.openWritingPipe(txAddress);
            radio.setPALevel(RF24_PA_LOW);
            radio.startListening();
            dernierEnvoi = millis();
        }
        bool receivePacket(received& Packet) {
            if (radio.available()) {
                radio.read(&Packet, sizeof(Packet));
                return true;
            }
            return false;
        }
        void sendPacket(const sent& packetSent) {
            radio.stopListening();
            radio.startWrite(&packetSent, sizeof(packetSent));
        }
        
        void alternateSend(const sent& packetSent, received& packetReceived) {
            unsigned long currentMillis = millis();
            if (currentMillis - dernierEnvoi >= interval) {
                sendPacket(packetSent);
                dernierEnvoi = currentMillis;
            } 
            if (!radio.isListening()) {
                radio.TXStandBy();
                radio.startListening();
            }
                receivePacket(packetReceived);
            
        }
        };

extern Emitor_receptor<ControlData, DroneData> radio;



#endif 

