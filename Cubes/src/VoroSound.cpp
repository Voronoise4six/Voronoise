#include "VoroSound.h"

#define SPEAKER_PIN D3
#define STANDBY_PIN D9
#define PWM_Ch 0
#define PWM_Resol 8

void sound_setup(){
    pinMode(STANDBY_PIN,OUTPUT);
    digitalWrite(STANDBY_PIN, LOW);
}

void notaAltavoz(int freq, int dur){
  digitalWrite(STANDBY_PIN, HIGH);
  ledcAttach(SPEAKER_PIN, freq, PWM_Resol);
  ledcChangeFrequency(SPEAKER_PIN, freq, PWM_Resol);
  ledcWrite(SPEAKER_PIN, 128);
  //ledcWrite(pin, freq);
  delay(dur);
  digitalWrite(STANDBY_PIN, LOW);
  ledcWrite(SPEAKER_PIN, 0);
}