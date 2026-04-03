#include "motorLOGIC.h"
#include "Pins.h"

// moteur class implementation
  moteur::moteur(const int& pin, const int& canal, float vitesse)
  : Moteur_pin(pin), canal(canal), vitesseMoteur(vitesse) {

  }
  void moteur::start() {
     pinMode(Moteur_pin, OUTPUT);
     ledcSetup(canal, 50, 14); // 50 Hz, 14-bit resolution
     ledcAttachPin(Moteur_pin, canal);
     ledcWrite(canal, 0); // Démarre à 0% de duty cycle (moteur éteint)
  }
  void moteur::armed() {
    estArmer=true;
  } 
  void moteur::unarmed() {
    estArmer=false;
  } 
  void moteur::vitCtrl(float vitesseRecue) {
 
    if (estArmer == false) {
      vitesseMoteur = vitesseMin;

    } else {
      vitesseMoteur = constrain(vitesseRecue, vitesseMin, vitesseMax);
    }
    // speed convertion to PWM value for the ESC 
    PWMvalue = 16384*(vitesseMoteur/20000.0); // 14-bit resolution
    ledcWrite(canal, PWMvalue);
  }


// mixMotor class implementation
   mixMotor::mixMotor() : m_ad(AV_D,12,1000), m_ag(AV_G,13,1000), m_dd(AR_D,14,1000), m_dg(AR_G,27,1000) {}

   void mixMotor::start() {
    m_ad.start(); m_ag.start(); m_dd.start(); m_dg.start();
   }
   //protection sup +
   void mixMotor::stopTout() {
        m_ad.unarmed(); m_ag.unarmed(); 
        m_dd.unarmed(); m_dg.unarmed();
    }
  void mixMotor::arming() {
    m_ad.armed(); m_ag.armed();
    m_dd.armed(); m_dg.armed();
    }
  void mixMotor::appliquer(float thr, float p, float r,  float y) {
    thr = constrain(thr, 1000,1600);p = constrain(p, 0,100);r = constrain(r, 0,100);y = constrain(y, 0,100);
    m_ag.vitCtrl(constrain(thr + p + r + y, 1000, 2000));
    m_ad.vitCtrl(constrain(thr + p - r - y, 1000, 2000)); 
    m_dg.vitCtrl(constrain(thr - p + r - y, 1000, 2000)); 
    m_dd.vitCtrl(constrain(thr - p - r + y, 1000, 2000));
  }

