#ifndef IMU_H
#define IMU_H
#include <Wire.h>
#include <Arduino.h>

class IMU {
  private:
    // MPU-6050 
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ; 
    // coordonnées quaternions pour le filtre de madgwick
    float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
    float offsetAccelX = 0, offsetAccelY = 0, offsetAccelZ = 0; // pour la calibration de l'accéléromètre
    //pour les calibrations imu :
    float erreurGyroX = 0, erreurGyroY = 0, erreurGyroZ = 0;
    // facteurs de conversions :
    const float GyroFC = 131.0;
    float AccelFC = 16384.0;
    const float AccMagFC = 600.0; // facteur de conversion pour la boussole, à ajuster selon les mesures
     float BETA = 0.1; // Coefficient pour le filtre complémentaire
    // boussole :

    
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

        magX = (int16_t)(Wire.read() << 8 | Wire.read());
        magY = (int16_t)(Wire.read() << 8 | Wire.read());
        magZ = (int16_t)(Wire.read() << 8 | Wire.read());
      

    }
  public :
  
  //calibration de l'imu
  void calibrerIMU();
    
    void wire_begin(int sda, int scl);

    void Beta_Modif();

   void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float DT);

    float MettreAjourmesures();

    float getAngleRoll();
    float getAnglePitch(); 
    float getAngleYaw() ;
};


extern IMU monIMU;

#endif // IMU_H
