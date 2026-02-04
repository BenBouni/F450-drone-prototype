#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "SPI.h"


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
IMU monIMU;
void setup(void) {
  Serial.begin(115200);
  monIMU.wire_begin(21, 22); //SDA, SCL pins
  monIMU.calibrerIMU();
  Serial.println("IMU Calibrating.");
  delay (2000);
  Serial.println("IMU Calibrated.");
}

void loop() {
  //obtention des données IMU
    float dt = monIMU.MettreAjourmesures();
    float angleRoll = monIMU.getAngleRoll();
    float anglePitch = monIMU.getAnglePitch();
    float angleYaw = monIMU.getAngleYaw();

    Serial.print("GS,");
    Serial.print(anglePitch);
    Serial.print(",");
    Serial.print(angleRoll);
    Serial.print(",");
    Serial.print(angleYaw);
    Serial.print(",");
    Serial.println(dt);
    delay(100);
}