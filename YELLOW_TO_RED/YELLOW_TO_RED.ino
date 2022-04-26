#include "FastLED.h"
#include<NoDelay.h>


#define NUM_LEDS 12
#define DATA_PIN 7
#define CLOCK_PIN 13

CRGB leds[NUM_LEDS];
void migajLED();
bool redColor = false;
noDelay migajLEDtime(500,migajLED);
void setup() {
  delay(2000);

  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
}
byte YELLOW = 30;
byte MINUS_COLOR = YELLOW / NUM_LEDS;
byte LED_ZAKRES = 12;

void loop()
{
  //migajLEDtime.update();
  FastLED.clear();
  

  for(int i = 0;i< LED_ZAKRES; i++)
  {
    leds[i] = CHSV(YELLOW - (MINUS_COLOR * i), 255, 255);
    
  }
  FastLED.show();

}

void migajLED()
{
  if(redColor)
  {
    FastLED.clear();
    FastLED.show();
    redColor = false;
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    redColor = true;
  }
  
}
