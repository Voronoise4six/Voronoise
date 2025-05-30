#include <Arduino.h>

#include <VoroConfig.h> //Lo mimso se puede quitar
#include <VoroLED.h>
#include <VoroSound.h>
#include <VoroComs.h>
#include <VoroIMU.h>
//#include "TripleIntQueue.h"

//#include <esp_now.h>
//#include <WiFi.h> 
//#include <esp_wifi.h>

////////////////////////////////////////////////////////////
////////////////////////// DEFINITIONS /////////////////////
////////////////////////////////////////////////////////////

int ID_CUBO=0; //ID de cada cubo
int MODO_CUBO=1; //el modo en el que está este cubo
int threshold_modo_golpe=4; //los modos 1 y 2 son de golpe
int threshold_modo_mantener=8; //los modos 3 y 4 son de mantener nota
int frecuencias[8]={220, 262, 294, 330, 392, 440, 523, 587}; //pentatonica menor de A

int cara_actual=0; //el modo en el que está este cubo
int cara_vieja=0; //el modo en el que está este cubo
int cara_golpe=0;

int nota_LED=0;
int modo_LED=0;
int cubo_LED=0;

bool dCancelAction = false;

int ShakeDetectado=0;


//lista de MACs de todos los cubos
uint8_t List_of_MAC_Addresses[][6] = {  //AÑADIR CADA CUBO O RECEPTOR
  {0x9C, 0x9E, 0x6E, 0x77, 0x99, 0xE0},  // cubo1 
  {0x8C, 0xBF, 0xEA, 0xCF, 0x76, 0x68},   // cubo2 
  {0x8c, 0xbf, 0xea, 0xcf, 0x75, 0xe8},  // cubo3 
  {0x8c, 0xbf, 0xea, 0xcf, 0x96, 0x58},    // cubo4 
  {0x8c, 0xbf, 0xea, 0xcf, 0x8e, 0xec},    // cubo5 
  {0x8c, 0xbf, 0xea, 0xcf, 0x8f, 0x20},    // cubo6 
  {0x64, 0xe8, 0x33, 0x86, 0x65, 0x80}    // receptor comunicaciones (un C3)

};
uint8_t n_direcciones=7; //A CAMBIAR a 6 o 7 (con portatil)

static TaskHandle_t procesar_LED_cola_handle;
static TaskHandle_t procesar_IMU_handle;

SimpleTripleFIFO myBuffer(32); //estaban a 1024, lo hemos bajado porque se usa menos con el dCancelAction
SimpleTripleFIFO ComsBuffer(32);


/////////////////////////////////////////////////////////////

volatile bool imuDataReady = false;
hw_timer_t *Timer_IMU = NULL;
void IRAM_ATTR IMU_periodic_reading(){
  imuDataReady=true;
}


void check_LEDs(void * pvParameters){
  for(;;){
    int i=0;
    if (myBuffer.pop(nota_LED, modo_LED, cubo_LED)) {
      LEDevent(nota_LED,modo_LED,cubo_LED);
      //Serial.print("sacado");
      i=1;
    }
    if (i==0){
      if (ComsBuffer.pop(nota_LED, modo_LED, cubo_LED)) {
      LEDevent(nota_LED,modo_LED,cubo_LED);
      //Serial.print("sacado2");
    }
    }

  }
}

void check_IMU(void * pvParameters){
  for (;;){
    if (imuDataReady) {
      IMU_loop(); 
      shaker_detection_routine();
      if (ShakeDetectado==1){
        MODO_CUBO+=1;
        if (MODO_CUBO>threshold_modo_mantener){MODO_CUBO=1;}
        PLACEHOLDER_EventoGolpe(7,MODO_CUBO);
        //myBuffer.push(7,MODO_CUBO,ID_CUBO);  //no lo hemos estado poniendo. Probarlo
        ShakeDetectado=0;
        notaAltavoz(frecuencias[MODO_CUBO-1], 150);
      }
      else{
        if (MODO_CUBO<=threshold_modo_golpe){
          print_IMU_values_golpe();
          if (cara_golpe!=0){
            PLACEHOLDER_EventoGolpe(cara_golpe,MODO_CUBO);
            dCancelAction = true;
            myBuffer.push(cara_golpe,MODO_CUBO,ID_CUBO);
            //LEDevent(cara_golpe,1,ID_CUBO);
            cara_golpe=0;
          }
        }
      else if (MODO_CUBO>threshold_modo_golpe){
        IMU_face_detection();
        if (cara_actual!=cara_vieja){
          if (cara_actual>99){
            myBuffer.push(0,MODO_CUBO,ID_CUBO);
            PLACEHOLDER_EventoGolpe(0,MODO_CUBO);
            //LEDevent(0,3,ID_CUBO); 
            //Serial.print("apago");
          }
          else{
            myBuffer.push(cara_actual,MODO_CUBO,ID_CUBO);
            PLACEHOLDER_EventoGolpe(cara_actual,MODO_CUBO);
            //LEDevent(cara_actual+1,3,ID_CUBO); 
            //Serial.print("enciendo");
          }
        }
      }
      }   
    }
  }
}


void setup() {
  Serial.begin(115200);
  delay(50);
  //Wifi_setup();
  //checkMacAddress();

  //setups
  Coms_setup();
  delay(50);
  sound_setup();
  delay(50);
  LED_setup();
  delay(50);
  IMU_setup();
  delay(50);
  //timer tarea IMU
  Timer_IMU=timerBegin(1000000);
  timerAttachInterrupt(Timer_IMU, &IMU_periodic_reading);
  timerAlarm(Timer_IMU, 5000, true, 0);

   xTaskCreate( //tarea princiapl de checkear la IMU. Mayor prioridad (2)
    check_IMU,
    "CheckIMU",
    8192,
    NULL,
    2,
    &procesar_IMU_handle
  );

  xTaskCreate( //tarea de comprobar LEDs, a menos prioridad que la IMU
    check_LEDs,
    "CheckLED",
    8192,
    NULL,
    1,
    &procesar_LED_cola_handle
  );
  delay(50);
  theaterChaseRainbow(25);
  setColorHSV(0,0,0);
  notaAltavoz(frecuencias[ID_CUBO-1], 150);
}



void loop() {
    vTaskDelete(NULL);

  //PLACEHOLDER_EventoGolpe(1,1);
  //  LEDevent(0,3,ID_CUBO); 
  //  delay(2000);
  //  LEDevent(cara_actual+1,3,ID_CUBO); 
  //  delay(2000);   
  //   //print_IMU_values();
  //delay(2);
}






