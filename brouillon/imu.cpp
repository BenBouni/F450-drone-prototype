#include <Arduino.h>
#include <Wire.h>
#include <math.h>


class IMU {
    private :
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float erreurGyroX, erreurGyroY, erreurGyroZ;
    float offsetAccelX, offsetAccelY, offsetAccelZ;
    float AccelFC, GyroFC;
    float dt;
    float tempsPrecedent;
    float pitch, roll, yaw;
    float alpha = 0.98; // complementary filter coefficient, to be tuned for better performance
    
    void lireDonneesBrutes() {
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // register to start reading the data from the sensor
        Wire.endTransmission(false); // end the transmission but keep the connection active to read the data
        Wire.requestFrom(0x68, 14, true); // request 14 bytes of data from the sensor, which correspond to the accelerometer and gyroscope measurements

        accelX = (int16_t)(Wire.read() << 8 | Wire.read());
        accelY = (int16_t)(Wire.read() << 8 | Wire.read());
        accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
        Wire.read(); Wire.read(); // ignore the temperature data, we won't use it for now
        gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

        dt = (millis() - tempsPrecedent) / 1000.0;
        tempsPrecedent = millis();
    }
    public :
    void calibrerIMU() {
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
        delay(2);  // little delay to space the measurements, to avoid any issues with the I2C bus or the sensor itself, and to get a more accurate average of the measurements for the calibration
    }

    // Biais gyro
    erreurGyroX = sommeGx / nbEchantillons;
    erreurGyroY = sommeGy / nbEchantillons;
    erreurGyroZ = sommeGz / nbEchantillons;

    // Biais accelero
    float accelX_moyen = sommeAx / nbEchantillons;
    float accelY_moyen = sommeAy / nbEchantillons;
    float accelZ_moyen = sommeAz / nbEchantillons;

    offsetAccelX = accelX_moyen;
    offsetAccelY = accelY_moyen;
    offsetAccelZ = accelZ_moyen;

}
 void update() {
    lireDonneesBrutes();

    accelX -= offsetAccelX;
    accelY -= offsetAccelY;
    accelZ -= offsetAccelZ;

    gyroX -= erreurGyroX;
    gyroY -= erreurGyroY;
    gyroZ -= erreurGyroZ;
    // Convert to physical units
    gyroX = (gyroX/GyroFC)*M_PI/180.0f
    gyroY = (gyroY/GyroFC)*M_PI/180.0f;
    gyroZ = (gyroZ/GyroFC)*M_PI/180.0f

    accelX = accelX/AccelFC;
    accelY = accelY/AccelFC;
    accelZ = accelZ/AccelFC;

    
     //complemary filter
        float accelRoll = atan2f(accelY, accelZ) * 180.0f / M_PI;
        float accelPitch = atan2f(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / M_PI;
        float accelYaw = atan2f(gyroZ, sqrt(gyroX * gyroX + gyroY * gyroY)) * 180.0f / M_PI;

        roll = alpha * (roll + gyroX * dt) + (1 - alpha) * accelRoll;
        pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * accelPitch;
        yaw = alpha * (yaw + gyroZ * dt) + (1 - alpha) * accelYaw;

 
}
    float getAngleRoll() const { return roll; }
    float getAnglePitch() const { return pitch; }
    float getAngleYaw() const { return yaw; }
};

void setup() {
  Serial.begin(115200);
  IMU monIMU;
  monIMU.wire_begin(4, 5); // SDA, SCL pins
  PinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // calibration in progress
    delay(5000);
    digitalWrite(2, LOW); // ready for flight
  monIMU.calibrerIMU();

}

void loop() {
  monIMU.update();
  Serial.print("GS,");

  Serial.print(monIMU.getAngleRoll(),",");

  Serial.print(monIMU.getAnglePitch(),",");

  Serial.println(monIMU.getAngleYaw());
  delay(100);
}