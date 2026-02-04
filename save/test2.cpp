#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>

struct ControlData {
   float thr; //1000 à 2000
   float yaw;
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
    // Angles filtrés (ce que le PID utilisera)
    float angleRoll = 0.0;
    float anglePitch = 0.0;
    float angleYaw = 0.0; // Le Yaw est uniquement géré par le Gyro

    // Gestion du temps pour le filtre (Calcul de Delta t)
    unsigned long tempsPrecedent = 0;
    const float ALPHA = 0.98; // Coefficient de pondération du filtre complémentaire
  void lireDonneesBrutes() {
        // Demande au MPU de commencer à nous envoyer les 6 valeurs brutes
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // Le premier registre où se trouve AccelX
        Wire.endTransmission(false); // Garde la connexion ouverte

        // Demande 14 octets de donnees brutes (Accel X,Y,Z, Temp, Gyro X,Y,Z)
        Wire.requestFrom(0x68, 14); // disambiguate overload

        // Lecture des données brutes (chaque valeur est sur 2 octets = un int)
        // Note : MPU-6050 stocke les données en 'High Byte' puis 'Low Byte'
        accelX = (int16_t)(Wire.read() << 8 | Wire.read()); 
        accelY = (int16_t)(Wire.read() << 8 | Wire.read());
        accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
        
        // pour ignorer Lecture de la température 
        Wire.read() << 8 | Wire.read(); // Skip Temp
        
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
      Wire.write(0x6B); //reveil
      Wire.write(0x00); //lecture
      
      Wire.endTransmission(true); // fin
      calibrerIMU();
      tempsPrecedent = micros();   
  }
  


  IMU() { 
      angleRoll = 0.0;
      anglePitch = 0.0;
      tempsPrecedent = millis();
    }
    
    float MettreAjourmesures() {
        lireDonneesBrutes();
        //données calibrées
        gyroX -= erreurGyroX;
        gyroY -= erreurGyroY;
        gyroZ -= erreurGyroZ;
        
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
        angleYaw += (gyroZ*dt ); // accel Z n'a pas de projection 
        return dt;
      } 

  float getRoll() { return angleRoll; }
  float getPitch() { return anglePitch; }
  float getYaw() { return angleYaw; }
          
};

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
        }        
    }
    bool sendPacket(const DroneData& packetDrone) { // send to GS
        dernierEnvoi = millis();
        radio.stopListening();
        radio.openWritingPipe(address);
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


class moteur {

  private :
  const int Moteur_pin;
  float vitesseMoteur;
  int vitesseMin = 1000;
  int vitesseMax = 2000;
  bool estArmer = false;

  public:
  moteur(const int& pin, float vitesse)
  : Moteur_pin(pin), vitesseMoteur(vitesse) {

  }
  void start() {
    pinMode(Moteur_pin, OUTPUT);
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
  }

};

class mixMotor {
  private:
  moteur m_ad;
  moteur m_ag;
  moteur m_dd;
  moteur m_dg;
 
  public : 
   mixMotor() : m_ad(12,1000), m_ag(13,1000), m_dd(14,1000), m_dg(27,1000) {}  

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
    m_ag.vitCtrl(thr + p + r + y);
    m_ad.vitCtrl(thr + p - r - y);
    m_dg.vitCtrl(thr - p + r - y);
    m_dd.vitCtrl(thr - p - r + y);
  }
};

class PID {
  private :
  float kp, ki, kd;
  float sumErreur = 0;
  float erreurPre = 0;

  public: 
  PID(float p, float i,float d) : kp(p), ki(i), kd(d) {}

  float calcErreur(float demand, float mesure, float dt) {
    float erreur = demand -  mesure ;
    
    float P = erreur * kp;
   
    sumErreur += erreur * dt ;
    if (sumErreur < -400) sumErreur = -400;
    if (sumErreur > 400) sumErreur = 400;
    float I = sumErreur *ki ;

    float D =0;
    if (dt > 0) D = kd * (erreur-erreurPre)/dt;
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
        mixer.appliquer(1500, 0, 0, 0); // stationnaire
    } else {
        // normal operation, do nothing
      }
    }
};

IMU monIMU;
PID PIDroll(1.0, 0.0, 0.0);
PID PIDpitch(1.0, 0.0, 0.0);
PID PIDyaw(1.0, 0.0, 0.0);
PID PIDthr(1.0, 0.0, 0.0);

float dt;
bool state_= false;

mixMotor monMix;
Emitor_receptor radio;
failsafe monFailsafe(1000, 10000); // 1 second timeout, 10 second critical loss timeout
#define SDA_PIN 4
#define SCL_PIN 5
unsigned long tempsCyclePre = 0;
const unsigned long PERIOD_CYCLE = 4000; // 250 Hz

void setup() {
  Serial.begin(115200);
  monIMU.wire_begin(SDA_PIN, SCL_PIN);
  radio.begin();
  monMix.start();
  monMix.arming();
}

void loop() {
  ControlData Packet; // receive from GS
  DroneData dronepacket; // send to GS
  static float r_order = 0;
  static float p_order = 0;
  static float y_order = 0;
  static float thr_order = 1000;
  static bool isArmed = false;
  static float dt = 0.004;

if (micros() - tempsCyclePre > PERIOD_CYCLE ){
  //mise a jour des mesures
  dt = monIMU.MettreAjourmesures(); 
  float mesureRoll = monIMU.getRoll();
  float mesurePitch = monIMU.getPitch();
  float mesureYaw = monIMU.getYaw();

    if (radio.receivePacket(Packet)) {
    //mise a jour commande
    r_order = Packet.roll;
    p_order = Packet.pitch;
    y_order = Packet.yaw;
    thr_order = Packet.thr;  
    isArmed = Packet.armed; 
    monFailsafe.updateSignalTime();
  } else {
    //stabilistion en cas de perte
    r_order = 0;
    p_order = 0;
    y_order = 0;
    monFailsafe.failsafeAction(monMix);
  }
      // calculate correction
    float Rollcorr = PIDroll.calcErreur(r_order, mesureRoll, dt );
    float Yawcorr = PIDyaw.calcErreur(y_order, mesureYaw, dt);
    float Pitchcorr = PIDpitch.calcErreur(p_order, mesurePitch, dt);
    if (isArmed == true) {
    monMix.arming();
     // apply motor mixing
     monMix.appliquer(thr_order, Pitchcorr, Rollcorr, Yawcorr );
  } else {
      monMix.stopTout();
  }
  // update GS
  tempsCyclePre = micros();
  dronepacket.batteryVoltage = 11.1; // Example voltage
  dronepacket.ActualPitch = monIMU.getPitch();
  dronepacket.ActualRoll = monIMU.getRoll();
  dronepacket.ActualYaw = monIMU.getYaw();
  radio.sendDroneData(dronepacket);
  //debug  
  Serial.print(monIMU.getRoll());
  Serial.println(monIMU.getPitch());
  Serial.println(monIMU.getYaw());    
  }
}
// note : l'obtention des mesures se fait en 1micros , les calculs et l'action se fait en 4micros