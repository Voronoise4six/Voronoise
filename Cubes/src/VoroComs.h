/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#ifndef VOROCOMS_H
#define VOROCOMS_H

#include <esp_now.h>
#include <WiFi.h> 
#include <esp_wifi.h>
#include "VoroConfig.h"

 
// Callback para notificar si el envío fue exitoso
//lo podremos quitar también, a menos que queramos reenviar ante fallo por alguna razon
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
 
// Callback para recibir datos
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len);
 
void PLACEHOLDER_EventoGolpe(int nota, int modo);
//comprobar la mac de este cubo y asignarle una ID. Se lee una vez metida la wifi en coms setup
void checkMacAddress();

void Wifi_setup();

void Coms_setup();

#endif /* VOROCOMS_H */