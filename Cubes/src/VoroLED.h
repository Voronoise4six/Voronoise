#ifndef VOROLED_H
#define VOROLED_H

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include "VoroConfig.h"


//#include <VoroConfig.h>

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait);
  
// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait);

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait);

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait);

//establecer un color, saturación y brillo
void setColorHSV(int hue, int sat, int val);

//desde apagado a maxima intensidad en un tiempo dado
void lightUp(int hue, int time);

//desde maxima intensidad hasta apagado en un tiempo dado
void lightDown(int hue, int time);

void colorWipeHSV(int hue, int sat, int val, int wait);


void transistoryWipe(int hue_fin, int hue_trans, int encendido_fin);


void LED_setup();


int hueCalcGolpe(int nota);

int hueCalcMantener();

//llamar a LEDevent al recibir comunicaciones de otro cubo o al detectar evento de este cubo
void LEDevent(int nota, int modo, int cubo);

#endif /* VOROCOMS_H */