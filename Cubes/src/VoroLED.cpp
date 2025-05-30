//#include <Adafruit_NeoPixel.h>
//#ifdef __AVR__
// #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
//#endif
#include "VoroLED.h"


#define LED_PIN D10
#define LED_COUNT 8

int cubo_influenciando[6]={0,0,0,0,0,0}; //solo 1 si esta en modo mantener y ademas está tocando una nota
double influencias_color[6]={0,0,0,0,0,0}; //en radianes
int color_propio=0;


#define DelayBreak(T) delay(T); if (dCancelAction) break;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a DelayBreak time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
      strip.show();                          //  Update strip to match
      DelayBreak(wait);                           //  Pause for a moment
    }
  }
  
// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a DelayBreak time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      DelayBreak(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass DelayBreak time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    DelayBreak(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass DelayBreak time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      DelayBreak(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

//establecer un color, saturación y brillo
void setColorHSV(int hue, int sat, int val) {
  for (int i = 0; i < strip.numPixels(); i++) {
    // Establece el color del píxel usando el valor de tono 'hue'
    strip.setPixelColor(i, strip.ColorHSV(hue, sat ,val));
  }
  strip.show(); // Actualiza la tira de LEDs
  //DelayBreak(wait);
}

//desde apagado a maxima intensidad en un tiempo dado
void lightUp(int hue, int time){
  for (int t=0; t<256;t++){
    for (int i = 0; i < strip.numPixels(); i++) {
      // Establece el color del píxel usando el valor de tono 'hue'
      strip.setPixelColor(i, strip.ColorHSV(hue, 255 ,t));
    }
    strip.show(); // Actualiza la tira de LEDs
    DelayBreak(time/256);
  }
}

//desde maxima intensidad hasta apagado en un tiempo dado
void lightDown(int hue, int time){
  for (int t=255; t>=0;t--){
    for (int i = 0; i < strip.numPixels(); i++) {
      // Establece el color del píxel usando el valor de tono 'hue'
      strip.setPixelColor(i, strip.ColorHSV(hue, 255 ,t));
    }
    strip.show(); // Actualiza la tira de LEDs
    DelayBreak(time/256);
    //break; o continue
  }
}

void colorWipeHSV(int hue, int sat, int val, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, strip.ColorHSV(hue, sat, val));         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    DelayBreak(wait);                           //  Pause for a moment
  }
}


void transistoryWipe(int hue_fin, int hue_trans, int encendido_fin){
  //setColorHSV(hue_init,255,255);
  //DelayBreak(500);
  colorWipeHSV(hue_trans, 255, 75, 15);
  //delay(50); 
  colorWipeHSV(hue_fin, 255, 255*encendido_fin, 30); // al color inicial
}


void LED_setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)  Define el techo de luminosidad
}


int hueCalcGolpe(int nota){
  double hue=65536/6*(nota-1);
  double hue_rad=hue/65536*2*PI;
  float trig_result=0;
  //int influenciando=-1*cubo_influenciando[ID_CUBO-1];
  int influenciando=0;
  float main_weight=0.6;  //A MODIFICAR
  float other_weight=0;
  float suma_senos=0;
  float suma_cosenos=0;

  //cuantos otros cubos influyen en el color y su peso
  for (int i=0;i<6;i++){
    influenciando+=cubo_influenciando[i];
  }
  if (influenciando==0) { //ningun cubo influye o solo esta el mismo en mantener con nota
    return hue;
  } 
  else{
    other_weight=(1-main_weight)/influenciando;
    suma_senos+=sin(hue_rad)*main_weight;
    suma_cosenos+=cos(hue_rad)*main_weight;
    for (int i=0;i<6;i++){
      if (cubo_influenciando[i]==1){ //si va muy lento con trigonometricas, lookup table o simplificar calculo
        suma_senos+=sin(influencias_color[i])*other_weight;
        suma_cosenos+=cos(influencias_color[i])*other_weight;
      }
    }
    trig_result=atan2(suma_senos,suma_cosenos);
    if (trig_result<0){trig_result+=2*PI;}
    return trig_result/(2*PI)*65536;
  }
}

int hueCalcMantener(){
  int influenciando=0;
  float main_weight=0.6; //A MODIFICAR
  float other_weight=0;
  float suma_senos=0;
  float suma_cosenos=0;
  float trig_result=0;

  for (int i=0;i<6;i++){
    influenciando+=cubo_influenciando[i];
  }
  if (influenciando==1 && cubo_influenciando[ID_CUBO-1]==1){return influencias_color[ID_CUBO-1]/(2*PI)*65536;} //no hay nada extra influyendo
  
  other_weight=(1-main_weight)/(influenciando-1);
  for (int i=0;i<6;i++){
    if (cubo_influenciando[i]==1 && ID_CUBO!=(i+1)){ //si va muy lento con trigonometricas, lookup table o simplificar calculo
      suma_senos+=sin(influencias_color[i])*other_weight;
      suma_cosenos+=cos(influencias_color[i])*other_weight;
    }
  }
  suma_senos+=sin(influencias_color[ID_CUBO-1])*main_weight;
  suma_cosenos+=cos(influencias_color[ID_CUBO-1])*main_weight;
  trig_result=atan2(suma_senos,suma_cosenos);
  if (trig_result<0){trig_result+=2*PI;}
  return trig_result/(2*PI)*65536;
}

void actualizarInfluencia(int nota, int cubo){
  if (nota==0 || nota==7){ //si no hay nota mantenida, desactivar
    cubo_influenciando[cubo-1]=0;
  }
  else{
    influencias_color[cubo-1]=2*PI/6*(nota-1);
    cubo_influenciando[cubo-1]=1;
  }
}

//llamar a LEDevent al recibir comunicaciones de otro cubo o al detectar evento de este cubo
void LEDevent(int nota, int modo, int cubo){
  dCancelAction = false;
  int hue=0;
  if (cubo==ID_CUBO){
    if (nota==7){ //hacer arcoiris y apagar
      actualizarInfluencia(nota, cubo);
      //rainbow(50);
      theaterChaseRainbow(15);
      strip.clear();
      strip.show();
    }
    else if (modo<=threshold_modo_golpe){ //flash de color
      //setColorHSV(hue, 255, 255, 50);
      lightDown(hueCalcGolpe(nota), 500);
    }
    else{ //mantener color y actualizar influencias
      actualizarInfluencia(nota, cubo);
      if (nota!=0){
        color_propio=hueCalcMantener();
        setColorHSV(color_propio, 255, 255);
      }
      else{
        //apagar LEDs si has recibido parar, nota 0
        strip.clear();
        strip.show();
      }
    }
  }
  else{
    if (nota==7){
      actualizarInfluencia(nota, cubo); //quitar la influencia del que acaba de cambiar de modo
      if (cubo_influenciando[ID_CUBO-1]==1){ //si este cubo esta emitiendo nota continuada, actualizar color
        color_propio=hueCalcMantener();
        setColorHSV(color_propio, 255, 255);
      }
    }
    else if (modo<=threshold_modo_golpe){ //wipe de color sin borrar el de base
      //colorWipe(hue, 50); ver como hacer una vuelta de ese color y luego volver al que estabamos
      if (cubo_influenciando[ID_CUBO-1]==1){transistoryWipe(color_propio,65536/6*(nota-1), 1);}//si el cubo estaba con un color, dar vuelta y volver al color
      else{transistoryWipe(color_propio,65536/6*(nota-1), 0);} //wipe pero se apaga despues
    }
    else{//añadir influencia y actualizar el color si este cubo esta en mantener nota y actuvo
      actualizarInfluencia(nota, cubo);
      //setColorHSV(hue, 255, 255, 50);
      if (cubo_influenciando[ID_CUBO-1]==1){ //si este cubo esta emitiendo nota continuada, actualizar color
        color_propio=hueCalcMantener();
        setColorHSV(color_propio, 255, 255);
      }
    }
  }
}

