#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>

struct dataPacket {  // Sent to drone
    float thr;
    float yaw;
    float pitch;
    float roll;
    float armed;
};
struct dataPacketDrone { // Received from drone
    float batteryVoltage;
    float anglePitch;
    float angleRoll;
    float angleYaw;
    float ActualThrottle;
};

class RadioEmitor_Receptor {
    private:
    const uint8_t channel;
    const uint64_t PipelineAddressTX;
    const uint64_t PipelineAddressRX;
    RF24 radio; // CE, CSN pins
public:
    RadioEmitor_Receptor(const uint8_t channel, const uint64_t PipelineAddressTX, const uint64_t PipelineAddressRX) : channel(channel), PipelineAddressTX(PipelineAddressTX), PipelineAddressRX(PipelineAddressRX), radio(4, 5) {}
    void begin() {
    if (!radio.begin()) {
        Serial.println("ERREUR : nRF24 non détecté ! Vérifie ton câblage SPI.");
        while (1); // Bloque ici si c'est mort
    }
    if (!radio.isChipConnected()) {
        Serial.println("ERREUR : nRF24 connecté mais puce non répondante !");
        while (1);
    }
    radio.setPALevel(RF24_PA_MIN); // Garde MIN pour les tests à 1m
    radio.setChannel(channel);
    radio.setDataRate(RF24_1MBPS);
    Serial.println("nRF24 OK.");
}
    //send packet to drone
    bool sendPacket(const dataPacket& packet) {
        radio.openWritingPipe(PipelineAddressTX);
        radio.stopListening();
        return radio.write(&packet, sizeof(packet));
    }
    bool radioWrite(const dataPacket& packet) {
        return radio.write(&packet, sizeof(packet));
    }
    //receive drone packet
    bool receivePacket( dataPacketDrone& packetDrone) {
        radio.openReadingPipe(1, PipelineAddressRX);
        radio.startListening();
        return true;
    }
    bool radioRead( dataPacketDrone& packetDrone) {
        if (radio.available()) {
            radio.read(&packetDrone, sizeof(packetDrone));
            return true;
        }
        return false;
    }
};

RadioEmitor_Receptor* RADIO;
dataPacketDrone dronepacket;
void setup(void) {
  Serial.begin(115200);
  RADIO = new RadioEmitor_Receptor(100, 0xE8E8F0F0E1LL, 0xE8E8F0F0E2LL);
    RADIO->begin();
    RADIO->receivePacket(dronepacket);
}

void loop() {

    dataPacketDrone packetDrone;

    if (RADIO->radioRead(packetDrone)) {
        Serial.print("GS,");
        Serial.print(packetDrone.batteryVoltage);
        Serial.print(",");
        Serial.print(packetDrone.anglePitch);
        Serial.print(",");
        Serial.print(packetDrone.angleRoll);
        Serial.print(",");
        Serial.print(packetDrone.angleYaw);
        Serial.print(",");
        Serial.println(packetDrone.ActualThrottle);
    }
    }