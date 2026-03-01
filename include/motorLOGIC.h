#ifndef MOTORLOGIC_H
#define MOTORLOGIC_H
#include <Wire.h>   
#include <math.h>
#include <Servo.h>
#include "data.h"
#include "config.h"

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
    // speed convertion to PWM value for the ESC 
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
   mixMotor() : m_ad(AV_D,12,1000), m_ag(AV_G,13,1000), m_dd(AR_D,14,1000), m_dg(AR_G,27,1000) {}  

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

class PID {
  private :
  float kp, ki, kd;
  float sumErreur = 0;
  float erreurPre = 0;
  unsigned long lastMicros = 0;
  float DT = 0.0001;
  const float DT_MAX = 0.001f; // 0.001 second, to avoid big drifts of the PID in case of code blocking or abberant mesures
  public: 
  PID(float p, float i,float d) : kp(p), ki(i), kd(d)  {}

  float dt_init() {
    unsigned long currentMicros = micros();
    
    DT = (currentMicros - lastMicros) / 1000000.0f; // conversion to seconds
    lastMicros = currentMicros;
    if (DT > DT_MAX) DT = DT_MAX ; // avoid big drifts of the PID in case of code blocking or abberant mesures
    return DT;
  }

  float calcErreur(float demand, float mesure) {
     
    if (isnan(mesure) || isinf(mesure)) {
    return 0.0f; // return 0 if mesure is invalid
    }
    DT = dt_init();    
    float erreur = demand - mesure;

    float P = erreur * kp;
   
    sumErreur += erreur * DT;
    // anti windup
    if (sumErreur < -400) sumErreur = -400;
    if (sumErreur > 400) sumErreur = 400;
    float I = sumErreur *ki ;

    float D =0;
    if (DT >= 0.0001f) D = kd * (erreur-erreurPre)/DT;
    erreurPre = erreur;
 // the correction
    return P + I + D; 
  }

};

extern mixMotor monMix;
extern PID PIDroll;
extern PID PIDpitch;    
extern PID PIDyaw;

#endif