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