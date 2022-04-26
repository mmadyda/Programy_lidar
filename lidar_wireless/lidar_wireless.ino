/*
   Ctrl+T auto format kodu
   ---------------------------------
   1. Download this sketch code to your Arduino board
   2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
   3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
*/


#include <RPLidar.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "FastLED.h"
#include<NoDelay.h>


#define CE_PIN 9
#define CSN_PIN 10

#define NUM_LEDS 12
#define LED_DATA_PIN 7

CRGB leds[NUM_LEDS];
void migajLED();
bool redColor = false;
byte YELLOW = 30;
byte MINUS_COLOR = YELLOW / NUM_LEDS;
noDelay migajLEDtime(500,migajLED);
//w słupkach zamieniamy adresy pipeRead z pipeWrite
const unsigned int pipeRead = 500;
const unsigned int pipeWrite = 800;

RF24 radio(CE_PIN, CSN_PIN);
float dane[2];
float msg[2];

SoftwareSerial mySerial(3, 4); // RX, TX LIDAR

RPLidar lidar;

#define RPLIDAR_MOTOR 2 // The PWM pin for control the speed of RPLIDAR's motor.

const float ODL_MIN = 0.6;
const float SKOK = 0.2;

float ZAKRESY[NUM_LEDS];


int CZULOSC = 1;  //USTAWIENIE CZUŁOŚCI
int liczba_skanow = 0;

unsigned long previousMillis = 0;
const long BLINK_INTERVAL = 500;   // interval at which to blink LED (milliseconds)
int ledState = LOW;
float min_dist = 999; //najbliższy zmierzony punkt
void dopisz()
{
  /*
    mySerial.print("kat: ");
    mySerial.print(angle);
    mySerial.print(" odleglosc: ");
    mySerial.print(metry);
    mySerial.print("   ");
  */
  mySerial.println(min_dist);
}
void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 1);//restartowanie silnika lidar
  delay(2000);
  analogWrite(RPLIDAR_MOTOR, 0);
  FastLED.addLeds<WS2811, LED_DATA_PIN, RGB>(leds, NUM_LEDS);
  for(int i = 0;i<NUM_LEDS;i++)
  {
    ZAKRESY[i] = ODL_MIN + (i*SKOK);
  }
  dane[0] = 0.0;
  
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_HIGH);           //ustaw wzmocnienie modułu radiowego na wysokie 
  radio.openWritingPipe(pipeWrite);
  radio.openReadingPipe(1, pipeRead);
  //adio.startListening();
  //radio.write(dane, sizeof(dane));
  // bind the RPLIDAR driver to the arduino hardware serial
  
  lidar.begin(Serial);
  mySerial.begin(115200);


  // set pin modes
  
  
}

void loop() {
  //FastLED.clear();
  radio.openWritingPipe(pipeWrite);
  radio.openReadingPipe(1, pipeRead);
  
  unsigned long currentMillis = millis();
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float metry = distance / 1000;
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement


    //if (angle > 270 or angle < 90)

    if (angle > 355 or angle < 5)
    {
      if (metry < min_dist and metry != 0)
      {
        min_dist = metry;
        
      }
    }


    if (startBit == 1)
    {
      dane[0] = min_dist;
      radio.stopListening();
      radio.write(dane, sizeof(dane));
      radio.startListening();
      liczba_skanow++;
      dopisz();
      if (liczba_skanow >= CZULOSC)
      {

        liczba_skanow = 0;



        if (min_dist <= ZAKRESY[0] )
        {
          migajLEDtime.update();
        }
        /*
        else if (min_dist > ODL_1 and min_dist <= ODL_2 )
        {
          mySerial.println("2");

        }
        else if (min_dist > ODL_2  and min_dist <= ODL_3 )
        {
          mySerial.println("3");

        }
        else if (min_dist > ODL_3 and min_dist <= ODL_4 )
        {
          mySerial.println("4");

        }
        else if (min_dist > ODL_4 and min_dist <= ODL_5 )
        {
          mySerial.println("5");

        }
        else if (min_dist > ODL_5 and min_dist <= ODL_6 )
        {
          mySerial.println("6");

        }
        else if (min_dist > ODL_6 and min_dist <= ODL_7 )
        {
          mySerial.println("7");

        }
        else if (min_dist > ODL_7 and min_dist <= ODL_8 )
        {
          mySerial.println("8");


        }
        */
        else if(min_dist > ZAKRESY[NUM_LEDS-1])
        {
          mySerial.println("daleko");
          FastLED.clear();
          FastLED.show();
        }
        min_dist = 999;
      }//if (startBit == 1)
    }//if(liczba_skanow >= CZULOSC)

   
  

  }// koniec if (IS_OK(lidar.waitPoint()))
  else
  {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
  if(radio.available())
  {
  radio.read(msg, sizeof(msg));
  }
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
