#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h> 

int threshold_modo_golpe=4; //los modos 1, 2 y 3 son de golpe
int threshold_modo_mantener=8; //los modos  4, 5 y 6 son de mantener nota. Igual a n_modos. IMPORTANTE

int notaCubos[6]={-1,-1,-1,-1,-1,-1}; //que nota esta sonando en cada cubo. -1 es sin nota
int ModoCubos[6]={1,1,1,1,1,1}; //en que modo estan los cubos. Todo a 1 al principio
int CuboSonando[6]={0,0,0,0,0,0}; //registra si la nota MIDI de ese cubo sigue activa
int millisSonando[6]={0,0,0,0,0,0};

//Lista Modos: 
//Golpe: 1.Guitarra 2.Piano Electrico 3.Bateria 4.Bajo  5.Sinte
//Mantener: 6.Organo  7.Orquesta  8.Sinte 9.Sinte(Octava alta)
int nota_base=69; //A4. Nota desde la que se calcula todo. En octava 4 (piano)
int octavas[8]={0, -1, 0, -1, -2, -1, -2, 0}; //cuantas octavas por debajo o por encima de la nota central
int intervalos[6]={0, 3, 5, 7, 10, 12}; //pentatonica menor
//int notasMIDI[6]; //almacenaba el mapeo de pruebas incial. Porbablemente podremos borrarla mas tarde
int mapaMIDI[8][6]; //almacena el mapeo de notas MIDI para cada modo. CAMBIAR EL PRIMER NUMERO SEGUN EL NUMERO DE MODOS
int mapaBateria[6]={36, 40, 42, 47, 48, 49};  //mapeo especial para el MIDI bateria AR70s Tight Kit Lite
int modo_bateria=3; //que modo es la bateria, del 1 al threshold_modo_mantener
//int mapaBajo[6]={57, 36, 38, 40, 43, 45};
//int modo_bajo=4;


///////////////////////////////////////////////////////////////////////////
//            MIDI
//////////////////////////////////////////////////////////////////////////

//abrir Hairless MIDI y enchufar a puerto LoopBe en MIDI Out

//byte noteON=0x09;
byte noteON_comb=0x90;
//byte noteOFF=0x08;
byte noteOFF_comb=0x80;

void MIDInoteON(byte channel, byte pitch, byte velocity){
  //Serial.write(noteON);
  Serial.write(noteON_comb | channel); //channel 0-15
  Serial.write(pitch);  
  Serial.write(velocity);  
  }

void MIDInoteOFF(byte channel, byte pitch, byte velocity){
  //Serial.write(noteOFF);
  Serial.write(noteOFF_comb | channel); //channel 0-15
  Serial.write(pitch);  
  Serial.write(velocity);  
  }

///////////////////////////////////////////////////////////////////
//              COMS
//////////////////////////////////////////////////////////////////

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

String success;
 
// Estructura de datos a recibir
typedef struct struct_message {
    uint8_t nota;
    uint8_t modo;
    uint8_t cubo;  
} struct_message;
 
struct_message incomingReadings;

// Callback para notificar si el envío fue exitoso
//lo podremos quitar también, a menos que queramos reenviar ante fallo por alguna razon
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nÚltimo estado de envío:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Envío exitoso" : "Error en el envío");
}

//cada modo debe ir ligado a un canal MIDI especifico. Por ahora asumimos que modo es igual a canal (modo 1 es channel 1 que es un 0)
void registrar_evento(int nota, int modo, int cubo){
  // Serial.print(nota);
  //   Serial.print(modo);
  // Serial.println(cubo);

  if (nota==0 || nota==7){ //callar cubo o cambiar de modo
    int flag=0;
    for (int i=0;i<6;i++){ //comprobar que ningun otro cubo estaba usando esa nota en ese modo para poder apagarla
      if (i!=cubo-1 && CuboSonando[i]==1 && ModoCubos[i]==ModoCubos[cubo-1] && notaCubos[i]==notaCubos[cubo-1]){
        flag=1;
      }
    }
    if (flag==0){
      if (nota==0){MIDInoteOFF(modo-1, mapaMIDI[modo-1][notaCubos[cubo-1]-1], 100);}
      else if (nota==7){
        MIDInoteOFF(modo-2, mapaMIDI[modo-2][notaCubos[cubo-1]-1], 100);} //-2 porque estoy enviando un modo nuevo
      } //no la usa otro cubo, podemos apagar
    //en todo caso, actualizamos valores para este cubo
    //notaCubos[cubo-1]=-1;
    CuboSonando[cubo-1]=0;
    if (nota==7){
       for (int i=0;i<6;i++){
         MIDInoteOFF(modo-2, mapaMIDI[modo-2][i], 100);
       }
      ModoCubos[cubo-1]=modo;} //cambiar de modo cuando es 7
  }
  else{
    if (CuboSonando[cubo-1]==1){
      MIDInoteOFF(modo-1, mapaMIDI[modo-1][notaCubos[cubo-1]-1], 100);
    }
    MIDInoteON(modo-1, mapaMIDI[modo-1][nota-1], 100);
    notaCubos[cubo-1]=nota;
    CuboSonando[cubo-1]=1;
    if (modo<=threshold_modo_golpe){millisSonando[cubo-1]=millis();}
  }
}

void checkNotas(){
  int tiempo_max_golpe_sonando=100; //cuanto tiempo se queda sonando un golpe. A ajustar
  int flag=0;
  int t=millis();
  for (int i=0;i<6;i++){
    if (CuboSonando[i]==1 && ModoCubos[i]<=threshold_modo_golpe && t-millisSonando[i]>tiempo_max_golpe_sonando){
      flag=0;
      for (int j=0;j<6;j++){
        if (i!=j && CuboSonando[j]==1 && ModoCubos[i]==ModoCubos[j] && notaCubos[i]==notaCubos[j] && t-millisSonando[j]<tiempo_max_golpe_sonando){
          flag=1; //hay un cubo tocando lo mismo pero todavia le queda un rato de sonar, no apago la nota
        }
      }
      if (flag==0){ //no hay coincidencias, puedo apagar la nota y actualizar valores
        MIDInoteOFF(ModoCubos[i]-1, mapaMIDI[ModoCubos[i]-1][notaCubos[i]-1], 100);
        notaCubos[i]=-1;
        CuboSonando[i]=0;
      }
    }
  }
}

// Callback para recibir datos
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  registrar_evento(incomingReadings.nota,incomingReadings.modo,incomingReadings.cubo);

  // Identificar de qué dispositivo viene la información
  //todo esto se puede quitar luego
  // Serial.print("Datos recibidos de: ");
  // for (int i = 0; i < 6; i++) {
  //   Serial.printf("%02X", mac->src_addr[i]);
  //   if (i < 5) Serial.print(":");
  // }
  // Serial.println();
  // Serial.print(incomingReadings.nota);
  // Serial.print(incomingReadings.modo);
  // Serial.println(incomingReadings.cubo);
}

void Wifi_setup(){
  WiFi.mode(WIFI_STA);
}

void Coms_setup() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  //checkMacAddress(); //para ver que cubo soy
 
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
 
  // Registrar dispositivos
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Agregar cada dispositivo a la lista de peers
  for (int i=0; i<n_direcciones-1; i++){
    memcpy(peerInfo.peer_addr, List_of_MAC_Addresses[i], 6);
    //Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
    //List_of_MAC_Addresses[i][0], List_of_MAC_Addresses[i][1], List_of_MAC_Addresses[i][2],
    //List_of_MAC_Addresses[i][3], List_of_MAC_Addresses[i][4], List_of_MAC_Addresses[i][5]);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      //Serial.print("Error al agregar peer");
      //Serial.println(i+1);
    }
  }
}
 
//////////////////////////////////////////////////////////////////


void MapeoMidi(){
  for (int i=0; i<threshold_modo_mantener;i++){
    for (int j=0;j<6;j++){
      if (i==modo_bateria-1){mapaMIDI[i][j]=mapaBateria[j];}
      //else if (i==modo_bajo-1){mapaMIDI[i][j]=mapaBajo[j];}
      else{mapaMIDI[i][j]=nota_base+12*octavas[i]+intervalos[j];}
    }
  }
}


void setup(){
  //for (int i=0;i<6;i++){notasMIDI[i]=nota_base+intervalos[i];}
  MapeoMidi();
  Serial.begin(115200);
  //delay(1000);
  Coms_setup();
}
 
void loop(){
  // MIDInoteON(3, 60, 100);
  // delay(1000);
  // MIDInoteOFF(3, 60, 100);
  // delay(1000);
  checkNotas();
}