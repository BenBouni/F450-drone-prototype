#include "motorLOGIC.h"


// Helper: convert microseconds high time to LEDC duty (14-bit, 50Hz)
static inline int micros_to_duty(unsigned long us) {
  const int RES = 14;
  const int MAX_DUTY = (1<<RES) - 1; // 16383
  const unsigned long PERIOD_US = 20000UL; // 20 ms
  long v = (long)roundf((float)us * (float)MAX_DUTY / (float)PERIOD_US);
  if (v < 0) v = 0;
  if (v > MAX_DUTY) v = MAX_DUTY;
  return (int)v;
}

// moteur class implementation
  moteur::moteur(const int& pin, const int& canal, float vitesse)
  : Moteur_pin(pin), canal(canal), vitesseMoteur(vitesse) {

  }
  void moteur::start() {
     pinMode(Moteur_pin, OUTPUT);
     ledcSetup(canal, 50, 14); // 50 Hz, 14-bit resolution
     ledcAttachPin(Moteur_pin, canal);
     ledcWrite(canal, 0); // Démarre a 0 de duty cycle (moteur éteint)
  }
  void moteur::armed() {
    estArmer=true;
  } 
  void moteur::unarmed() {
    estArmer=false;
  } 

  void moteur::stopImmediate() {
   // mark unarmed and write minimum throttle immediately to ESC
   estArmer = false;
   vitesseMoteur = vitesseMin;
   PWMvalue = micros_to_duty((unsigned long)vitesseMoteur); // convert µs to 14-bit duty
   ledcWrite(canal, PWMvalue);
  }
  void moteur::vitCtrl(float vitesseRecue) {
 
    if (estArmer == false) {
      vitesseMoteur = vitesseMin;

    } else {
      vitesseMoteur = constrain(vitesseRecue, vitesseMin, vitesseMax);
    }
    // speed conversion to PWM value for the ESC (vitesseMoteur is in µs)
    PWMvalue = micros_to_duty((unsigned long)vitesseMoteur); // 14-bit resolution
    ledcWrite(canal, PWMvalue);
  }


// mixMotor class implementation
   mixMotor::mixMotor() : m_ad(AV_D,12,1000), m_ag(AV_G,13,1000), m_dd(AR_D,14,1000), m_dg(AR_G,27,1000) {}

   void mixMotor::start() {
    m_ad.start(); m_ag.start(); m_dd.start(); m_dg.start();
   }
   //protection sup +
   void mixMotor::stopTout() {
       // Ensure motors are immediately set to minimum PWM and marked unarmed for reliable failsafe
       m_ad.stopImmediate(); m_ag.stopImmediate();
       m_dd.stopImmediate(); m_dg.stopImmediate();
    }
  void mixMotor::arming() {
    m_ad.armed(); m_ag.armed();
    m_dd.armed(); m_dg.armed();
    }
  void mixMotor::appliquer(float thr, float p, float r,  float y) {
    thr = constrain(thr, 1100,1600);p = constrain(p, -100,100);r = constrain(r, -100,100);y = constrain(y, -100,100);
    m_ag.vitCtrl(constrain(thr + p + r + y, 1100, 2000));
    m_ad.vitCtrl(constrain(thr + p - r - y, 1100, 2000)); 
    m_dg.vitCtrl(constrain(thr - p + r - y, 1100, 2000)); 
    m_dd.vitCtrl(constrain(thr - p - r + y, 1100, 2000));
  }

