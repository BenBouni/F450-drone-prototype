#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>

class IMU {
  private:
    // MPU-6050 
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ; 
    //pour les calibrations imu :
    float erreurGyroX = 0, erreurGyroY = 0, erreurGyroZ = 0;
    // facteurs de conversions :
    const float GyroFC = 131.0;
    const float AccelFC = 16384.0;
    const float ALPHA = 0.98; // Coefficient pour le filtre complémentaire
    // Angles filtrés (ce que le PID utilisera)
    float angleRoll = 0.0;
    float anglePitch = 0.0;
    float angleYaw = 0.0;
    unsigned long tempsPrecedent;

    void lireDonneesBrutes() {
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // Adresse du registre de début des données
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 14, true); // Demander 14 octets

        accelX = (int16_t)(Wire.read() << 8 | Wire.read());
        accelY = (int16_t)(Wire.read() << 8 | Wire.read());
        accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
        Wire.read(); Wire.read(); // Ignorer Temp
        gyroX = (int16_t)(Wire.read() << 8 | Wire.read()); 
        gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
    }
  public :
  
  //calibration de l'imu
  void calibrerIMU() {
        float sommeX = 0, sommeY = 0, sommeZ = 0;
        for(int i = 0; i < 500; i++) {
            lireDonneesBrutes();
            sommeX += gyroX;  
            sommeY += gyroY;
            sommeZ += gyroZ;
        }
        erreurGyroX = sommeX / 500;
        erreurGyroY = sommeY / 500;
        erreurGyroZ = sommeZ / 500;
    }  
    //communication
    void wire_begin(int sda, int scl) {
      Wire.begin(sda, scl);
      Wire.beginTransmission(0x68); //commmencer
        Wire.write(0x6B); //lecture registre power management
        Wire.write(0x00);    //mettre à 0 pour sortir du mode veille
        Wire.endTransmission(true); // fin
        calibrerIMU();
        tempsPrecedent = millis();
    }
    
    // Met à jour les mesures et retourne le delta temps
    float MettreAjourmesures() {
        lireDonneesBrutes();
        //données calibrées
        gyroX -= erreurGyroX;
        gyroY -= erreurGyroY;
        gyroZ -= erreurGyroZ;
        // Conversion en unités physiques
        gyroX = gyroX/GyroFC;
        gyroY = gyroY/GyroFC;
        gyroZ = gyroZ/GyroFC;

        accelX = accelX/AccelFC;
        accelY = accelY/AccelFC;
        accelZ = accelZ/AccelFC;

        unsigned long tempsActuel = micros();
        float diffTemps = tempsActuel - tempsPrecedent;
        tempsPrecedent = tempsActuel;
        float dt = diffTemps/1000000.0;
        

        //projection de l'accel : 
        float angleAccelRoll = atan2(accelY,accelZ) * 180/M_PI;
        float angleAccelPitch = atan2(-accelX,sqrt(accelY*accelY+accelZ*accelZ)) * 180/M_PI;

        //nos angles finals : 
        angleRoll = ALPHA*(gyroX*dt + angleRoll) +(1-ALPHA)*angleAccelRoll;
        anglePitch = ALPHA*(gyroY*dt + anglePitch) +(1-ALPHA)*angleAccelPitch;
        angleYaw += gyroZ*dt ; // accel Z n'a pas de projection 
        return dt;

      }       
        float getAngleRoll() { return angleRoll; }
        float getAnglePitch() { return anglePitch; }
        float getAngleYaw() { return angleYaw; }
};


struct dataPacket {  // received from GS
    float thr;
    float yaw;
    float pitch;
    float roll;
    float armed;
};

struct dataPacketDrone { // sent to GS
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
    //send packet to GS 
    bool sendPacket(const dataPacketDrone& packet) {
        radio.openWritingPipe(PipelineAddressTX);
        radio.stopListening();
        return radio.write(&packet, sizeof(packet));
    }
    bool radioWrite(const dataPacketDrone& packetDrone) {
        return radio.write(&packetDrone, sizeof(packetDrone));
    }
    bool receivePacket( dataPacket& packet) {
        radio.openReadingPipe(1, PipelineAddressRX);
        radio.startListening();
        return true;
    }
    bool radioRead( dataPacket& packet) {
        if (radio.available()) {
            radio.read(&packet, sizeof(packet));
            return true;
        }
        return false;
    }
};
class failsafeMechanism {
  private:
    unsigned long lastSignalTime;
    const unsigned long timeoutmin; // in milliseconds
    const unsigned long timeoutmax ;
  public:
    failsafeMechanism(unsigned long timeout, unsigned long maxTimeout) : timeoutmin(timeout), timeoutmax(maxTimeout), lastSignalTime(millis()) {}
    void updateSignalTime() {
        lastSignalTime = millis();
    }
    bool temporaryLoss() {
        return (millis() - lastSignalTime) > timeoutmin;
    }
    bool criticalLoss() {
        return (millis() - lastSignalTime) > timeoutmax;
}
};

failsafeMechanism failsafe(1000, 10000); // 1 second timeout, 10 second critical loss timeout
RadioEmitor_Receptor* RADIO;
#define LED_BUILTIN 2
dataPacket packet;
dataPacketDrone dronepacket;
IMU monIMU;


void setup(void) {
  Serial.begin(115200);
  monIMU.wire_begin(21, 22); //SDA, SCL pins
  monIMU.calibrerIMU();
  Serial.println("IMU Calibrating.");
  delay (2000);
  Serial.println("IMU Calibrated.");
  pinMode(LED_BUILTIN, OUTPUT);
  RADIO = new RadioEmitor_Receptor(100, 0xE8E8F0F0E2LL, 0xE8E8F0F0E1LL);
  RADIO->begin();
  RADIO->sendPacket(dronepacket);
  Serial.println("Setup complete, waiting to send packets...");
}

        void loop() {
        //obtention des données IMU
    float dt = monIMU.MettreAjourmesures();
    float angleRoll = monIMU.getAngleRoll();
    float anglePitch = monIMU.getAnglePitch();
    float angleYaw = monIMU.getAngleYaw();

    // Send the packet
    dataPacketDrone packetDrone;
    packetDrone.batteryVoltage = 11.1; // Example voltage
    packetDrone.anglePitch = anglePitch;
    packetDrone.angleRoll = angleRoll;
    packetDrone.angleYaw = angleYaw;
    packetDrone.ActualThrottle = 1500; // Example throttle

    //filsafe implmentation


if (RADIO->sendPacket(packetDrone)) {
        digitalWrite(LED_BUILTIN, HIGH); // Indicate successful send
        Serial.println("Packet sent successfully:");
        failsafe.updateSignalTime(); // Update last signal time on successful send
    } else if (failsafe.temporaryLoss()) {
        digitalWrite(LED_BUILTIN, LOW); // Indicate failed send
        Serial.println("Failed to send packet.");
        thr
    }

if (failsafe.criticalLoss()) {
  Serial.println("Critical loss detected!");
}

   

    delay(10); // Wait before next loop
}
