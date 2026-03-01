#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>
#include <atomic>
#include <climits>
#include <Servo.h>

SemaphoreHandle_t xMutex;
void Radiocore(void * Pvparameter);

struct ControlData {
  float thr; //1000 à 1600
  float yaw; // 0 à 100
  float pitch;
  float roll;     
  bool armed;      
};

struct DroneData {
  float batteryVoltage;
  float ActualPitch;
  float ActualRoll;
  float ActualYaw;
};
 
struct Order {
  std::atomic <float> thr;
  std::atomic <float> pitch;
  std::atomic <float> roll;
  std::atomic <float> yaw;
  std::atomic <bool> armed;
};

struct telemerie {
    std::atomic <float> batteryVoltage;
    std::atomic <float> ActualPitch;
    std::atomic <float> ActualRoll;
    std::atomic <float> ActualYaw;
    };

  ControlData Packet; // receive from GS
  DroneData dronepacket; // send to GS
  Order order;
  telemerie tele;

class IMU {
  private:
    // MPU-6050 
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ; 
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

       
      
    }
  public :
  
  //calibration de l'imu
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

    void Beta_Modif() { // to modify the BETA parameter of the madgwick filter in real time from the terminal (tuning)
    if (Serial.available() > 0) {
        float newBeta = Serial.parseFloat(); 
        if (newBeta > 0) {
            BETA = newBeta;
            Serial.print("Nouveau BETA : ");
            Serial.println(BETA, 4);
        }
    }
}

   void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float DT) {
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
    float getAngleRoll() { return angleRoll; }
    float getAnglePitch() { return anglePitch; }  
    float getAngleYaw() { return angleYaw; }
};

IMU monIMU;

class Emitor_receptor { // Récepteur RF24
  private:
    RF24 radio;
    const byte address[6] = "00001";
    const unsigned long interval = 100; // Intervalle d'envoi en ms
    unsigned long dernierEnvoi;
  public:
    Emitor_receptor() : radio(21, 22) {} // CE, CSN pins
    void begin() {
      radio.begin();
      radio.openReadingPipe(0, address);
      radio.setPALevel(RF24_PA_HIGH);
      radio.startListening();
      dernierEnvoi = millis();
    }

    bool receivePacket(ControlData& packet) { // receive from GS 
        if (radio.available()) {
            radio.read(&packet, sizeof(packet));
            return true;
        } else return false;        
    }
    bool sendPacket(const DroneData& packetDrone) { // send to GS
        dernierEnvoi = millis();
        radio.stopListening();
        bool success = radio.write(&packetDrone, sizeof(packetDrone));
        radio.startListening();
        return success;
     }
    void sendDroneData(const DroneData& packetDrone) {
        unsigned long currentMillis = millis();
        if (currentMillis - dernierEnvoi >= interval) {
            sendPacket(packetDrone);
            dernierEnvoi = currentMillis;
        }
    } 
};
Emitor_receptor radio;


class moteur {
  private :
  const int Moteur_pin;
  const int canal;
  float vitesseMoteur;
  int PWMvalue;
  int vitesseMin = 1000;
  int vitesseMax = 2000;
  bool estArmer = false;
  
  public:
  moteur(const int& pin, const int& canal, float vitesse)
  : Moteur_pin(pin), canal(canal), vitesseMoteur(vitesse) {

  }
  void start() {
     pinMode(Moteur_pin, OUTPUT);
     ledcSetup(canal, 50, 14); // 50 Hz, 14-bit resolution
     ledcAttachPin(Moteur_pin, canal);
     ledcWrite(canal, 0); // Démarre à 0% de duty cycle (moteur éteint)
  }
  void armed() {
    estArmer=true;
  } 
  void unarmed() {
    estArmer=false;
  } 
  void vitCtrl(float vitesseRecue) {
 
    if (estArmer == false) {
      vitesseMoteur = vitesseMin;

    } else {
      vitesseMoteur = constrain(vitesseRecue, vitesseMin, vitesseMax);
    }
    // Convertir la vitesse en microsecondes pour le signal PWM
    PWMvalue = 16384*(vitesseMoteur/20000.0); // 14-bit resolution
    ledcWrite(canal, PWMvalue);

  }
};

class mixMotor {
  private:
  moteur m_ad;
  moteur m_ag;
  moteur m_dd;
  moteur m_dg;
 
  public : 
   mixMotor() : m_ad(12,1,1000), m_ag(13,2,1000), m_dd(14,3,1000), m_dg(27,4,1000) {}  

   void start() {
    m_ad.start(); m_ag.start(); m_dd.start(); m_dg.start();
   }
   //protection sup +
   void stopTout() {
        m_ad.unarmed(); m_ag.unarmed(); 
        m_dd.unarmed(); m_dg.unarmed();
    }

  void arming() {
    m_ad.armed(); m_ag.armed();
    m_dd.armed(); m_dg.armed();
    
    }

  void appliquer(float thr, float p, float r,  float y) {
    thr = constrain(thr, 1000,1600);p = constrain(p, 0,100);r = constrain(r, 0,100);y = constrain(y, 0,100);
    m_ag.vitCtrl(constrain(thr + p + r + y, 1000, 2000));
    m_ad.vitCtrl(constrain(thr + p - r - y, 1000, 2000)); 
    m_dg.vitCtrl(constrain(thr - p + r - y, 1000, 2000)); 
    m_dd.vitCtrl(constrain(thr - p - r + y, 1000, 2000));
  }

};
mixMotor monMix;

class PID {
  private :
  float kp, ki, kd;
  float sumErreur = 0;
  float erreurPre = 0;

  public: 
  PID(float p, float i,float d) : kp(p), ki(i), kd(d) {}

  float calcErreur(float demand, float mesure, float dt) {

    if (isnan(mesure) || isinf(mesure)) {
    return 0.0f; // Retourne 0 si mesure invalide
    }
    float erreur = demand - mesure;

    float P = erreur * kp;
   
    sumErreur += erreur * dt ;
    if (sumErreur < -400) sumErreur = -400;
    if (sumErreur > 400) sumErreur = 400;
    float I = sumErreur *ki ;

    float D =0;
    if (dt > 0.0001f) D = kd * (erreur-erreurPre)/dt;
    erreurPre = erreur;

    return P + I + D; 
  }

};

class failsafe {
  private:
    unsigned long lastSignalTime;
    const unsigned long timeoutmin; // in milliseconds
    const unsigned long timeoutmax ; // in milliseconds
   
  public:
    failsafe(unsigned long timeoutmin, unsigned long timeoutmaxs) : timeoutmin(timeoutmin), timeoutmax(timeoutmaxs) {
      lastSignalTime = millis();
    }

    void updateSignalTime() { // to be called when a valid signal is received
        lastSignalTime = millis();
    }

    bool temporaryLoss() {
      if (millis() - lastSignalTime > timeoutmin)
      return true;
      return false;
    } 
    bool criticalLoss() {
      if (millis() - lastSignalTime > timeoutmax)
      return true;
      return false;
    }
    void failsafeAction(mixMotor& mixer) {
      if (criticalLoss() ) {
        mixer.stopTout();
      } else if (temporaryLoss() ) {
        if (xSemaphoreTake(xMutex, 0) == pdTRUE){
          order.thr = 1300.0f; // Cut throttle
          order.pitch = 0.0f;
          order.roll = 0.0f;
          order.yaw = 0.0f;
          order.armed = false; // Disarm the drone
          xSemaphoreGive(xMutex);
        }; // stationnaire
    } else {
        // normal operation, do nothing
      }
    }
};


failsafe monFailsafe(1000, 10000);

class update_data {
  private :
  std::atomic <unsigned long> dernier_update {0};
  const unsigned long delai_update;
  std::atomic <bool> updated{false};
  public :
  update_data(unsigned long delaiupdate) : delai_update(delaiupdate) {}

  void data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED) {
   if (radio.receivePacket(Packet)) {
     THR = Packet.thr;
     PITCH = Packet.pitch;
     ROLL = Packet.roll;
     YAW = Packet.yaw;
     ARMED = Packet.armed;
    dernier_update = millis();
    monFailsafe.updateSignalTime();
   } 
   if (millis()-dernier_update > delai_update) {
    updated = false;
   } else updated = true; 
  }
  bool UPDATED() const { return updated.load(); }
};



PID PIDroll(1.0, 0.0, 0.0);
PID PIDpitch(1.0, 0.0, 0.0);
PID PIDyaw(1.0, 0.0, 0.0);
PID PIDthr(1.0, 0.0, 0.0);

float dt;

update_data data(500);

#define SDA_PIN 4
#define SCL_PIN 5
#define LED_PIN 2
unsigned long tempsCyclePre = 0;
const unsigned long PERIOD_CYCLE = 4000; // 250 Hz



void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  monIMU.wire_begin(SDA_PIN, SCL_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // calibration en cours
  delay(5000);  
  digitalWrite(LED_PIN, LOW); // pret pour le vol
  monMix.start();
  monMix.arming();
  xMutex = xSemaphoreCreateMutex();
      xTaskCreatePinnedToCore(
        Radiocore,
        "radioCORE",
        10000,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() {
  static unsigned long last_arm;
  static bool isArmed = false;
  static float r_order, p_order, y_order, thr_order;
   unsigned long currentTime = millis();

if (micros() - tempsCyclePre > PERIOD_CYCLE ){
  //mise a jour des mesures

//mise a jour commande
if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
    r_order = order.roll;
    p_order = order.pitch;
    y_order = order.yaw;
    thr_order = order.thr;

    if(order.armed && !isArmed) {
      if (millis() - last_arm > 500 ) {isArmed = true; last_arm = millis();}
    } else if (!order.armed && isArmed) {if (millis() - last_arm > 500 ) {isArmed = false; last_arm = millis();}}
    xSemaphoreGive(xMutex);
  } else {
    //stabilistion en cas de perte
    r_order = 0;
    p_order = 0;
    y_order = 0;
    monFailsafe.failsafeAction(monMix);
  }  
  dt = monIMU.MettreAjourmesures(); 
  float mesureRoll = monIMU.getAngleRoll();
  float mesurePitch = monIMU.getAnglePitch();
  float mesureYaw = monIMU.getAngleYaw();
      // calculate correction
    float Rollcorr = PIDroll.calcErreur(r_order, mesureRoll, dt );
    float Yawcorr = PIDyaw.calcErreur(y_order, mesureYaw, dt);
    float Pitchcorr = PIDpitch.calcErreur(p_order, mesurePitch, dt);

    if (order.armed.load() == true) {
     // apply motor mixing
     monMix.appliquer(order.thr, Pitchcorr, Rollcorr, Yawcorr );
  } else {
      monMix.stopTout();
  }
  // prepare data to send to GS
  tele.ActualPitch = monIMU.getAnglePitch();
  tele.ActualRoll = monIMU.getAngleRoll();
  tele.ActualYaw = monIMU.getAngleYaw();
  tele.batteryVoltage = 11.1; // Example voltage

  tempsCyclePre = micros();
  if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
  dronepacket.batteryVoltage = tele.batteryVoltage; // Example voltage
  dronepacket.ActualPitch = tele.ActualPitch;
  dronepacket.ActualRoll = tele.ActualRoll;
  dronepacket.ActualYaw = tele.ActualYaw;
  xSemaphoreGive(xMutex);
    }
  //debug  
  //Serial.print(monIMU.getRoll());
  //Serial.println(monIMU.getPitch());
  //Serial.println(monIMU.getYaw());    
  }
}


void Radiocore(void * Pvparameter) {
   radio.begin();
   for(;;){
    radio.receivePacket(Packet); 
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {            
       data.data(order.thr, order.pitch, order.roll, order.yaw, order.armed);
      xSemaphoreGive(xMutex);
      }
      radio.sendDroneData(dronepacket);
      vTaskDelay(4 / portTICK_PERIOD_MS);
   }
}

 
//    TODO LIST :
// all the code is executed in a loop with a period of 4ms on core 1 in parallel with the reception and sending of data by radio on core 0
// still need to tune the PID and the BETA parameter of the madgwick filter, and to test the failsafe in case of loss of signal from the GS, and to test the whole system in flight.
// might separate the tasks of everything and pu each task on a different loop with different frequencies (for example the PID loop at 4ms, the radio communication at 125Hz, and captors detection at 125Hz) to optimize the performance of the system.
// needto study and add mahnetometer to stabilize the yaw 
// still need more documentation and comments in the code to explain the different parts of the code and the logic behind it, and to make it more readable and maintainable. 


//    FEATURES TO ADD LATER:
// will also add some safety features like a kill switch, and a buzzer to alert the user in case of critical failure or low battery.
// will also add some features like a GPS module for position hold and return to home, and a barometer for altitude hold.
