//ustawic plytke arduino micro


#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 9); // RX, TX
#include "FastLED.h"

String metry = "";
#define NUM_LEDS 12
#define LED_DATA_PIN 10

byte LED_SWIEC = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long BLINK_INTERVAL = 500;   // interval at which to blink LED (milliseconds)
int ledState = LOW;
bool MIGAJ = false;
CRGB leds[NUM_LEDS];
void migajLED();
bool redColor = true;
byte YELLOW = 30;
byte MINUS_COLOR = YELLOW / NUM_LEDS;
void migajLED();

void setup() {
  // put your setup code here, to run once:
  FastLED.addLeds<WS2811, LED_DATA_PIN, RGB>(leds, NUM_LEDS);

  Serial.begin(115200);
  mySerial.begin(115200);
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(1000);
  FastLED.clear();
  FastLED.show();



}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  MIGAJ = false;
  if (mySerial.available())
  {
    byte NEW_LED_SWIEC = 255;
    delay(5);  // wait for all 4 bytes
    NEW_LED_SWIEC = mySerial.read();
    if (NEW_LED_SWIEC <= NUM_LEDS || NEW_LED_SWIEC == 99)
    {
      LED_SWIEC = NEW_LED_SWIEC;
    }

  }
  if (LED_SWIEC == 99)
  {
    MIGAJ = true;
  }
  else if (LED_SWIEC == 0)
  {
    FastLED.clear();
  }
  else
  {
    FastLED.clear();
    for (int i = 0; i < LED_SWIEC; i++)
    {

      leds[i] = CHSV(YELLOW - (MINUS_COLOR * (NUM_LEDS - i)), 255, 255);

    }
  }






  migajLED(MIGAJ);
  Serial.println(LED_SWIEC); // printing the result to the serial monitor
  FastLED.show();
}

void migajLED(bool miganie)
{
  if (currentMillis - previousMillis >= BLINK_INTERVAL) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (miganie)
    {
      if (redColor)
      {
        FastLED.clear();
        redColor = false;
      }
      else
      {
        fill_solid(leds, NUM_LEDS, CRGB::Red);

        redColor = true;
      }
      //FastLED.show();
    }

  }
}
