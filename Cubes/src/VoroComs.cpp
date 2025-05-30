/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/


#include "VoroLED.h" //para actualizar LEDs al recibir mensaje
#include "VoroComs.h"

String success;
 
// Estructura de datos para enviar
typedef struct struct_message {
    uint8_t nota;
    uint8_t modo;
    uint8_t cubo;  
} struct_message;
 
struct_message datos_a_enviar;
struct_message incomingReadings;
 
// Callback para notificar si el envío fue exitoso
//lo podremos quitar también, a menos que queramos reenviar ante fallo por alguna razon
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nÚltimo estado de envío:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Envío exitoso" : "Error en el envío");
}
 
// Callback para recibir datos
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //LEDevent(incomingReadings.nota,incomingReadings.modo,incomingReadings.cubo);
  dCancelAction=true;
  ComsBuffer.push(incomingReadings.nota,incomingReadings.modo,incomingReadings.cubo);
  
  
  // Identificar de qué dispositivo viene la información
  //todo esto se puede quitar luego
  Serial.print("Datos recibidos de: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.print(incomingReadings.nota);
  Serial.print(incomingReadings.modo);
  Serial.println(incomingReadings.cubo);
}
 
void PLACEHOLDER_EventoGolpe(int nota, int modo) { //PLACEHOLDER. 
  //Asignar los datos a enviar y mandarlos cuando hay un evento
  datos_a_enviar.nota = nota;
  datos_a_enviar.modo = modo;
  datos_a_enviar.cubo = ID_CUBO;
  Serial.print(nota);
  Serial.print(modo);
  Serial.println(ID_CUBO);

  for (int i=0;i<n_direcciones;i++){
    if (i!=ID_CUBO-1){
      esp_now_send(List_of_MAC_Addresses[i], (uint8_t *) &datos_a_enviar, sizeof(datos_a_enviar));
    }}
}
 
//comprobar la mac de este cubo y asignarle una ID. Se lee una vez metida la wifi en coms setup
void checkMacAddress(){
  int flagMac=0;
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
     Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                   baseMac[0], baseMac[1], baseMac[2],
                   baseMac[3], baseMac[4], baseMac[5]);
      for (int i=0;i<n_direcciones;i++){        
        for (int j=0;j<6;j++){
          if (baseMac[j]==List_of_MAC_Addresses[i][j]){
            flagMac++;
          }
        }
        if (flagMac==6){
          ID_CUBO=i+1;
        }
        flagMac=0;
      }
  } 
   else {
     Serial.println("Failed to read MAC address");
   }
}

void Wifi_setup(){
  WiFi.mode(WIFI_STA);
}

void Coms_setup() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  checkMacAddress(); //para ver que cubo soy
 
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
 
  // Registrar dispositivos
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Agregar cada dispositivo a la lista de peers
  for (int i=0; i<n_direcciones; i++){
    if (i!=(ID_CUBO-1)){
      memcpy(peerInfo.peer_addr, List_of_MAC_Addresses[i], 6);
      Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
      List_of_MAC_Addresses[i][0], List_of_MAC_Addresses[i][1], List_of_MAC_Addresses[i][2],
      List_of_MAC_Addresses[i][3], List_of_MAC_Addresses[i][4], List_of_MAC_Addresses[i][5]);
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.print("Error al agregar peer");
        Serial.println(i+1);
      }
    }
  }
}