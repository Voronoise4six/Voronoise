#ifndef VOROCONFIG_H
#define VOROCONFIG_H

#include <stdint.h>
#include <esp_wifi.h>
#include "SimpleTripleFIFO.h"

extern int ID_CUBO; //ID de cada cubo
extern int MODO_CUBO; //el modo en el que está este cubo
extern int threshold_modo_golpe; //los modos 1 y 2 son de golpe
extern int threshold_modo_mantener; //los modos 3 y 4 son de mantener nota
extern int frecuencias[8]; //pentatonica menor de A

extern int cara_actual;
extern int cara_vieja;
extern int cara_golpe;

extern int ShakeDetectado;

extern bool dCancelAction;

//lista de MACs de todos los cubos
extern uint8_t List_of_MAC_Addresses[][6];

extern uint8_t n_direcciones; //A CAMBIAR a 6 o 7 (con portatil)

extern SimpleTripleFIFO myBuffer;
extern SimpleTripleFIFO ComsBuffer;


#endif /* VOROCONFIG_H */