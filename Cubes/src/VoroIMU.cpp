#include "VoroIMU.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


#define INTERRUPT_PIN D8  // use pin 2 on Arduino Uno & most boards

#define HISTORY_MEAN 10

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int aaReal_x[HISTORY_MEAN];
int aaReal_y[HISTORY_MEAN];
int aaReal_z[HISTORY_MEAN];
int sumX=0;
int sumY=0;
int sumZ=0;
const float faceThreshold = 0.85; // Umbral de confianza para caras (85%)
const float edgeThreshold = 0.60; // Umbral para aristas (60%)

const float normals[6][3] = {   
  {1, 0, 0},   // Cara +X
  {-1, 0, 0},  // Cara -X
  {0, 1, 0},   // Cara +Y
  {0, -1, 0},  // Cara -Y
  {0, 0, 1},   // Cara +Z
  {0, 0, -1}   // Cara -Z
};
const char* faceNames[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

// Normales aproximadas de las 12 aristas (promedio de las caras adyacentes)
const float edgeNormals[12][3] = { 
  {1, 1, 0},   {-1, 1, 0},   {-1, -1, 0},   {1, -1, 0},  // Aristas en el plano XY
  {1, 0, 1},   {-1, 0, 1},   {-1, 0, -1},   {1, 0, -1},  // Aristas en el plano XZ
  {0, 1, 1},   {0, -1, 1},   {0, -1, -1},   {0, 1, -1}   // Aristas en el plano YZ
};
const char* edgeNames[12] = {
  "+X+Y", "-X+Y", "-X-Y", "+X-Y",
  "+X+Z", "-X+Z", "-X-Z", "+X-Z",
  "+Y+Z", "-Y+Z", "-Y-Z", "+Y-Z"
};
//int accelVal[3];      // Valores actuales del acelerómetro
int prevaccelVal[3];  // Valores previos del acelerómetro

int aa_x[HISTORY_MEAN];
int aa_y[HISTORY_MEAN];
int aa_z[HISTORY_MEAN];

int accelSqrt[HISTORY_MEAN];
int indiceMaxlocal = 0;
float accelX_prev = 0, accelY_prev = 0, accelZ_prev = 0, alpha = 0.1;//alpha = 0.197;//alpha = 0.862;
float accelX_agit_integral = 0, accelY_agit_integral = 0, accelZ_agit_integral = 0, betha = 0.01;//alpha = 0.197;//alpha = 0.862;
float accelX_agit_avg = 0,accelY_agit_avg = 0,accelZ_agit_avg = 0;

float accelX_buffer[HISTORY_MEAN] = {0};
float accelY_buffer[HISTORY_MEAN] = {0};
float accelZ_buffer[HISTORY_MEAN] = {0};
float sumXreal = 0, sumYreal = 0, sumZreal = 0;
int buffer_index  = 0;

int shaker = 0;
int prev_val_ypr[3] = {150, 150, 150};
int suma_ypr[3] = {0, 0, 0};

int Offset_Calibrados[][6]={ //calibrados con el codigo de ejemplo MPU_ZERO de la libreria IC2_MPU
  {-5815,-5909, 9771, 108, -82, 32},
  {4927, -5299, 10475, 130, 103, 16},
  {-4225, -5909, 9107, 88, -89, 21},
  {3437, -6897, 9381, 72, 81, -10},
  {-6295, -5299, 9107, -12, 32, -2},
  {-4577, -5967, 10367, 113, -63, -14}};


// --- Parámetros ajustables ---
#define my_fabsf(x) ((x) < 0 ? -(x) : (x))
#define DT 0.001f                // tiempo entre muestras en segundos (1 ms)
//Siempre meter el x.1f
#define ANGULAR_THRESH 80.1f      // umbral de velocidad angular (rad/ms). Fuerza bestia que hay que meterle. Minimo 80.1f o asi. Original 100.1f
#define MIN_SHAKES 25            // mínimo de eventos para detectar agitado. Ciclos para poder cambiar. Minimo 25. Original 30
#define WINDOW_SIZE 1000         // ventana de 1 segundo (1000 muestras)

static Quaternion prev_q = {1.0f, 0.0f, 0.0f, 0.0f}; // estado anterior
static bool first_sample = true;

static int shake_events = 0;
static int sample_counter = 0;


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void IMU_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    //comntado porque si no la IMU no tira
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(Offset_Calibrados[ID_CUBO][0]); // 1688 factory default for my test chip
    mpu.setYAccelOffset(Offset_Calibrados[ID_CUBO][1]); // 1688 factory default for my test chip
    mpu.setZAccelOffset(Offset_Calibrados[ID_CUBO][2]); // 1688 factory default for my test chip
    mpu.setXGyroOffset(Offset_Calibrados[ID_CUBO][3]);
    mpu.setYGyroOffset(Offset_Calibrados[ID_CUBO][4]);
    mpu.setZGyroOffset(Offset_Calibrados[ID_CUBO][5]);
    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        //mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}


void IMU_loop(){  //por alguna razon no funciona si la metes en la libreria
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            
    }
}  

void print_IMU_values(){
    Serial.print(aaReal.x);
    Serial.print(" ");
    Serial.print(aaReal.y);
    Serial.print(" ");
    Serial.print(aaReal.z);
    Serial.print(" ");
    Serial.println(sqrt(aaReal.z*aaReal.z+aaReal.x*aaReal.x+aaReal.y*aaReal.y));
}

////////////////////////////////////////////////////////////////////////////////////////




void IMU_face_detection(){

// Restar el valor más viejo
sumX -= aaReal_x[HISTORY_MEAN-1];
sumY -= aaReal_y[HISTORY_MEAN-1];
sumZ -= aaReal_z[HISTORY_MEAN-1];

// Desplazar valores
for(int i = HISTORY_MEAN-1; i > 0; i--) {
    aaReal_x[i] = aaReal_x[i-1];
    aaReal_y[i] = aaReal_y[i-1];
    aaReal_z[i] = aaReal_z[i-1];
}

// Añadir nuevo valor
aaReal_x[0] = aa.x;    //aaReal es para sin gravedad, gravity para gravedad,aa todo
aaReal_y[0] = aa.y;
aaReal_z[0] = aa.z;

// Sumar el nuevo valor
sumX += aa.x;
sumY += aa.y;
sumZ += aa.z;

// Calcular medias
float avgX = sumX / HISTORY_MEAN;
float avgY = sumY / HISTORY_MEAN;
float avgZ = sumZ / HISTORY_MEAN;


//  // Calculamos el vector de aceleración promedio con un filtro de media móvil
//  float avgX = (aaReal.x + aaReal_x[0]) / 2.0;
//  float avgY = (aaReal.y + aaReal_y[0]) / 2.0;
//  float avgZ = (aaReal.z + aaReal_z[0]) / 2.0;



 // Normalizamos el vector de aceleración
 float norm = sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
 if (norm > 0.01) {  // Evitamos divisiones por cero
   avgX /= norm;
   avgY /= norm;
   avgZ /= norm;
 }

 // Detectamos la posición
 int faceIndex = detectFace(avgX, avgY, avgZ, faceThreshold);
 int edgeIndex = (faceIndex == -1) ? detectEdge(avgX, avgY, avgZ, edgeThreshold) : -1;

 // Mostrar resultado en el puerto serie
//  Serial.print(avgX);
//  Serial.print(" ");
//  Serial.print(avgY);
//  Serial.print(" ");
//  Serial.print(avgZ);
//  Serial.println(" ");

 cara_vieja=cara_actual;
 if (faceIndex != -1) {
   //Serial.print("Face: ");
   //Serial.println(faceNames[faceIndex]); // Cara identificada
   cara_actual=faceIndex+1;
 } else if (edgeIndex != -1) {
   //Serial.print("Edge: ");
   //Serial.println(edgeNames[edgeIndex]); // Arista identificada
   cara_actual=100+edgeIndex;
 } else {
   //Serial.println("Other"); // Ninguna cara ni arista detectada
 }


//   for(int i = HISTORY_MEAN - 1; i > 0; i--) {
//     aaReal_x[i] = aaReal_x[i-1];
//     aaReal_y[i] = aaReal_y[i-1];
//     aaReal_z[i] = aaReal_z[i-1];
// }

//  aaReal_x[0] = aaReal.x;
//   aaReal_y[0] = aaReal.y;
//   aaReal_z[0] = aaReal.z;

}


int detectFace(float ax, float ay, float az, float threshold) {
  int detectedFace = -1;
  float maxProjection = 0;

  for (int i = 0; i < 6; i++) {
    // Producto punto con la normal de la cara
    float projection = ax * normals[i][0] + ay * normals[i][1] + az * normals[i][2];
    if (projection > maxProjection && projection > threshold) {
      maxProjection = projection;
      detectedFace = i;
    }
  }

  return detectedFace;
}

// Detecta la arista más alineada con el vector de aceleración
int detectEdge(float ax, float ay, float az, float threshold) {
  int detectedEdge = -1;
  float maxProjection = 0;

  for (int i = 0; i < 12; i++) {
    // Producto punto con la normal de la arista
    float projection = ax * edgeNormals[i][0] + ay * edgeNormals[i][1] + az * edgeNormals[i][2];
    if (projection > maxProjection && projection > threshold) {
      maxProjection = projection;
      detectedEdge = i;
    }
  }

  return detectedEdge;
}
  

/////////////////////////////////////////////

////////////////////////////////////////movidas de control de impactos y agitar

void actualizar_acumuladores(float accelX_prev, float accelY_prev, float accelZ_prev) {
  sumXreal -= accelX_buffer[buffer_index ];
  sumYreal -= accelY_buffer[buffer_index ];
  sumZreal -= accelZ_buffer[buffer_index ];

  accelX_buffer[buffer_index ] = accelX_prev;
  accelY_buffer[buffer_index ] = accelY_prev;
  accelZ_buffer[buffer_index ] = accelZ_prev;

  sumXreal += accelX_prev;
  sumYreal += accelY_prev;
  sumZreal += accelZ_prev;

  buffer_index  = (buffer_index  + 1) % HISTORY_MEAN;
}

void actualizar_agitacion_integrada(float accelX_agit, float betha) {
  float dt = 0.0005;
  accelX_agit_integral += accelX_agit * dt;
  // accelY_agit_integral += accelY_agit * dt;
  // accelZ_agit_integral += accelZ_agit * dt;
  accelX_agit_integral *= (1.0 - 0.001);

  //a 5000 cortde de agitar

  accelX_agit_avg += betha * (accelX_agit_integral - accelX_agit_avg);
  // accelY_agit_avg += betha * (accelY_agit_integral - accelY_agit_avg);
  // accelZ_agit_avg += betha * (accelZ_agit_integral - accelZ_agit_avg);
  
}

// Aproximación de arccos para x en [-1, 1]
float my_acosf(float x) {
  float negate = x < 0 ? 1.0f : 0.0f;
  x = my_fabsf(x);
  float ret = -0.0187293f;
  ret = ret * x + 0.0742610f;
  ret = ret * x - 0.2121144f;
  ret = ret * x + 1.5707288f;
  ret = ret * sqrtf(1.0f - x);
  return ret - 2 * negate * ret;
}

Quaternion normalize(Quaternion q) {
  float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (norm == 0) norm = 1.0f;  // evitar división por cero
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

// Función para producto punto
float quat_dot(Quaternion q1, Quaternion q2) {
  return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
}

// Ángulo entre cuaterniones
float quat_angle_between(Quaternion q1, Quaternion q2) {
  float dot = quat_dot(q1, q2);
  if (dot > 1.0f) dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;
  return 2.0f * my_acosf(my_fabsf(dot));
}

// Lógica de detección, a llamar cada 1 ms
bool update_and_check_shake(Quaternion q) {
  if (first_sample) {
      prev_q = q;
      first_sample = false;
      return false;
  }

  float angle = quat_angle_between(prev_q, q);
  float angular_speed = angle / DT;

  if (angular_speed > ANGULAR_THRESH) {
      shake_events++;
  }

  prev_q = q;
  sample_counter++;

  if (sample_counter >= WINDOW_SIZE) {
      bool is_shaking = shake_events >= MIN_SHAKES;

      // reset para próxima ventana
      shake_events = 0;
      sample_counter = 0;

      return is_shaking;
  }

  return false;
}

//////////// fin funciones movidas de impactos y agitar

void shaker_detection_routine(){
  if(shaker==0){
   Quaternion q_ = q;
   q_ = normalize(q_);
   if (update_and_check_shake(q_)) {
    //Aqui se cambia de modo
    shaker=2000;
    ShakeDetectado=1;
    }
  }else{shaker-=1;}

    //Serial.print(shaker,0);
  //  Serial.print(accelX_prev,0);
  //  Serial.print(" ");
  //  Serial.print(accelY_prev,0);
  //  Serial.print(" ");
  //  Serial.print(accelZ_prev,0);
  //  Serial.print(" ");
  // Serial.print(" ");

  //  Serial.print(q.w*100,0);
  //  Serial.print(" ");
  //  Serial.print(q.x*100,0);
  //  Serial.print(" ");
  //  Serial.print(q.y*100,0);
  //  Serial.print(" ");
  //  Serial.print(q.z*100,0);
  //  Serial.print(" ");
}


void print_IMU_values_golpe(){

  // Aplicación del filtro paso bajo
  accelX_prev = accelX_prev + alpha * (aaReal.x - accelX_prev);
  accelY_prev = accelY_prev + alpha * (aaReal.y - accelY_prev);
  accelZ_prev = accelZ_prev + alpha * (aaReal.z - accelZ_prev);
  actualizar_acumuladores( accelX_prev,  accelY_prev,  accelZ_prev);
  

  // Desplazar los valores anteriores
  for (int i = 1; i < 5; i++) {
    accelSqrt[i-1] = accelSqrt[i];
  }
   accelSqrt[4]=sqrt(accelZ_prev*accelZ_prev+accelX_prev*accelX_prev+accelY_prev*accelY_prev);
  
   actualizar_agitacion_integrada( accelSqrt[4], betha);


  //  Serial.print(accelSqrt[4],0);
  //  Serial.print(" ");
   
  //  Serial.print(accelX_agit_avg,0);
  //   Serial.print(" ");
  //   Serial.print(accelSqrt[4],0);
  //   Serial.print(" ");

    if (indiceMaxlocal >= 4) {
      if (accelSqrt[2] > accelSqrt[1] && accelSqrt[2] > accelSqrt[3]) {
        if (accelSqrt[2] > 8000){  //EL NUMERO ES EL THRESHOLD DE DETECTAR GOLPE. ORIGINALMENTE Y BIEN 8000
          int distXp = accelSqrt[2]-sumXreal/HISTORY_MEAN;
          int distXm = accelSqrt[2]+sumXreal/HISTORY_MEAN;
          int distYp = accelSqrt[2]-sumYreal/HISTORY_MEAN;
          int distYm = accelSqrt[2]+sumYreal/HISTORY_MEAN;
          int distZp = accelSqrt[2]-sumZreal/HISTORY_MEAN;
          int distZm = accelSqrt[2]+sumZreal/HISTORY_MEAN;
          int valores[6] = {
            abs(distXp),
            abs(distXm),
            abs(distYp),
            abs(distYm),
            abs(distZp),
            abs(distZm)
          };
          int menor = valores[0];
          int indiceMenor = 0;
          for (int i = 1; i < 6; i++) { //he cambiado i=1 a i=0
            if (valores[i] < menor) {
              menor = valores[i];
              indiceMenor = i;
            }
          }
          //mter aqui lo que sea de las caras para el golpe.
          cara_golpe=indiceMenor+1;
          // Serial.print("Índice del menor valor absoluto: ");
          //Serial.println(indiceMenor+1);
      }
        //Serial.print("Máximo local detectado en el eje X: ");
        //Serial.println(accelSqrt[2]);
        indiceMaxlocal=-60; //A MODIFICAR. ES CUANTO ESPERAR ENTRE GOLPE Y GOLPE. ORIGINAL -15. Va bien con -60
        // Realizar acciones adicionales si es necesario

      }else{
        //Serial.println(0);
    }
  }
    indiceMaxlocal++;
    // Serial.print(aaReal.x);
    // Serial.print(" ");
    // Serial.print(aaReal.y);
    // Serial.print(" ");
    // Serial.print(aaReal.z);
    // Serial.print(" ");
    // Serial.println(sqrt(aaReal.z*aaReal.z+aaReal.x*aaReal.x+aaReal.y*aaReal.y));

}

////////////////////////////////////////////////////////////////////////////////////////