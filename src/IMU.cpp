#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "SPI.h"


class IMU {
  private:
    // MPU-6050 
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ; 
    // coordonnées quaternions pour le filtre de madgwick
    float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
    
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
    void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float DT) {
        // Implémentation du filtre de Madgwick pour fusionner les données gyro et accel
        float qDot0, qDot1, qDot2, qDot3; 
        float diffT = 1.0f / 250.0f; // pour une mise à jour à 250Hz
        qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz); 
        qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) ;
        qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) ;
        qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) ;
        // Correction de la direction de la gravité 
        float recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Calcul du gradient de correction
        float s0, s1, s2, s3;
        float fx , fy , fz ; // Erreur entre la direction mesurée et la direction estimée de la gravité
        // projection de l'erreur sur les axes de l'IMU par rapport au dimension des quaternions
        fx = 2.0f * (q1 * q3 - q0 * q2) - ax; 
        fy = 2.0f * (q0 * q1 + q2 * q3) - ay;
        fz = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
        // Calcul du gradient de correction
        s0 = -2.0f * (q2 * fx - q1 * fy) ; 
        s1 = 2.0f * (q3 * fx + q0 * fy - 2.0f * q1 * fz) ;
        s2 = 2.0f * (q0 * fx - q3 * fy - 2.0f * q2 * fz) ;
        s3 = 2.0f * (q1 * fx + q2 * fy) ;
        // Normalisation du gradient
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        // Appliquer la correction
        float beta = 0.1f; // Gain de convergence
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
        // Intégrer pour obtenir les nouveaux quaternions
        q0 += qDot0 * DT ; // dt = approx 1/250s pour une mise à jour à 250Hz
        q1 += qDot1 * DT ;
        q2 += qDot2 * DT ;
        q3 += qDot3 * DT ;
        // Normalisation des quaternions
        recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        // Calcul des angles d'Euler à partir des quaternions
        angleRoll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
        anglePitch = asinf(2.0f * (constrain(q0 * q2 - q1 * q3, -1.0f, 1.0f))) * 180.0f / M_PI;
        angleYaw = atan2f(2.0f * (q0 * q3 +  q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
    }
    float MettreAjourmesures() {
        lireDonneesBrutes();
        //données calibrées
        gyroX -= erreurGyroX;
        gyroY -= erreurGyroY;
        gyroZ -= erreurGyroZ;
        // Conversion en unités physiques
        gyroX = (gyroX/GyroFC)*M_PI/180.0f; // Convertir en radians/s
        gyroY = (gyroY/GyroFC)*M_PI/180.0f;
        gyroZ = (gyroZ/GyroFC)*M_PI/180.0f;

        accelX = accelX/AccelFC;
        accelY = accelY/AccelFC;
        accelZ = accelZ/AccelFC;

        unsigned long tempsActuel = micros();
        float diffTemps = tempsActuel - tempsPrecedent;
        tempsPrecedent = tempsActuel;
        float dt = diffTemps/1000000.0;
        dt = max(0.0001f, dt); // éviter les divisions par zéro dans le filtre de Madgwick
        madgwickUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
 // Retourner les données brutes calibrées pour le filtre de Madgwick
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