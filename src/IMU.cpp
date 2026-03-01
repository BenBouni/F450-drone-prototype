#include "IMU.h"

IMU monIMU;

void IMU::wire_begin(int sda, int scl) {
    Wire.begin(sda, scl);
    Wire.beginTransmission(0x68); // Adresse I2C du MPU-6050
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // Mettre le MPU-6050 en mode actif
    Wire.endTransmission(true);
}

void IMU::calibrerIMU() {
    float sommeGx=0, sommeGy=0, sommeGz=0;
    float sommeAx=0, sommeAy=0, sommeAz=0;
    const int nbEchantillons = 500;

    for(int i = 0; i < nbEchantillons; i++) {
        lireDonneesBrutes();
        sommeGx += gyroX;
        sommeGy += gyroY;
        sommeGz += gyroZ;
        sommeAx += accelX;
        sommeAy += accelY;
        sommeAz += accelZ;
        delay(2);  // Petit délai pour espacer les mesures
    }

    // Biais gyro
    erreurGyroX = sommeGx / nbEchantillons;
    erreurGyroY = sommeGy / nbEchantillons;
    erreurGyroZ = sommeGz / nbEchantillons;

    // Calcul des moyennes accéléro
    float accelX_moyen = sommeAx / nbEchantillons;
    float accelY_moyen = sommeAy / nbEchantillons;
    float accelZ_moyen = sommeAz / nbEchantillons;

    // Ajustement du facteur d'échelle (gain)
    float accelMagnitude = sqrt(accelX_moyen * accelX_moyen + accelY_moyen * accelY_moyen + accelZ_moyen * accelZ_moyen);
    AccelFC = accelMagnitude / 16384.0f;

    // Offsets accéléro (on suppose qu'à l'arrêt, la mesure devrait être (0,0,1g) après conversion)
    // On les stocke en LSB pour les soustraire avant conversion
    offsetAccelX = accelX_moyen;
    offsetAccelY = accelY_moyen;
    offsetAccelZ = accelZ_moyen - AccelFC * 16384.0f; // On retire 1g en LSB
}

void IMU::Beta_Modif() {
    if (Serial.available() > 0) {
        float newBeta = Serial.parseFloat(); 
        if (newBeta > 0) {
            BETA = newBeta;
            Serial.print("Nouveau BETA : ");
            Serial.println(BETA, 4);
        }
    }
}
void IMU::madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float DT) {
    float recipNorm;
    float qDot0, qDot1, qDot2, qDot3;
    float s0, s1, s2, s3;

    // Taux de rotation (rad/s) -> dérivée des quaternions
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Normaliser les mesures d'accélération
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) return; // éviter les divisions par zéro
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.8f && recipNorm < 1.2f) { // éviter les mesures aberrantes
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    } else {
        return; // Ignorer les mesures d'accélération non valides
    }

    // Calcul de l'erreur entre la direction estimée de la gravité et la mesure
    float fx = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float fy = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float fz = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    // Calcul du gradient (J_g^T * f) selon l'algorithme de Madgwick
    s0 = -2.0f * q2 * fx + 2.0f * q1 * fy;                 // -2*q2*fx + 2*q1*fy
    s1 =  2.0f * q3 * fx + 2.0f * q0 * fy - 4.0f * q1 * fz; // 2*q3*fx + 2*q0*fy - 4*q1*fz
    s2 = -2.0f * q0 * fx + 2.0f * q3 * fy - 4.0f * q2 * fz; // -2*q0*fx + 2*q3*fy - 4*q2*fz
    s3 =  2.0f * q1 * fx + 2.0f * q2 * fy;                 // 2*q1*fx + 2*q2*fy

    // Normalisation du gradient
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Appliquer la correction (étape de descente de gradient)
    qDot0 -= BETA * s0;
    qDot1 -= BETA * s1;
    qDot2 -= BETA * s2;
    qDot3 -= BETA * s3;

    // Intégration
    q0 += qDot0 * DT;
    q1 += qDot1 * DT;
    q2 += qDot2 * DT;
    q3 += qDot3 * DT;

    // Normalisation des quaternions
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Calcul des angles d'Euler
    angleRoll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    anglePitch = asinf(2.0f * constrain(q0 * q2 - q1 * q3, -1.0f, 1.0f)) * 180.0f / M_PI;
    angleYaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}

float IMU::MettreAjourmesures() {
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

        accelX -= offsetAccelX;
        accelY -= offsetAccelY;
        accelZ -= offsetAccelZ;

        unsigned long tempsActuel = micros();
        float diffTemps = tempsActuel - tempsPrecedent;
        tempsPrecedent = tempsActuel;
        float dt = diffTemps/1000000.0;
        dt = max(0.0001f, dt); // éviter les divisions par zéro dans le filtre de Madgwick
        madgwickUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
 // Retourner les données brutes calibrées pour le filtre de Madgwick
       return dt;
      }   
    
    float IMU::getAngleRoll() { return angleRoll; }
    float IMU::getAnglePitch() { return anglePitch; }  
    float IMU::getAngleYaw() { return angleYaw; }
