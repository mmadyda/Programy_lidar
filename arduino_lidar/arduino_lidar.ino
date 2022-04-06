/*
   Ctrl+T auto format kodu
   ---------------------------------
   1. Download this sketch code to your Arduino board
   2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
   3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
*/


#include <RPLidar.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX LIDAR

RPLidar lidar;

#define RPLIDAR_MOTOR 4 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal

float ODL_1 = 0.6;
float ODL_2 = 0.8;
float ODL_3 = 1.0;
float ODL_4 = 1.2;
float ODL_5 = 1.4;
float ODL_6 = 1.6;
float ODL_7 = 1.8;
float ODL_8 = 2.0;

int zakres_1 = 0;
int zakres_2 = 0;
int zakres_3 = 0;
int zakres_4 = 0;
int zakres_5 = 0;
int zakres_6 = 0;
int zakres_7 = 0;
int zakres_8 = 0;

int liczba_skanow = 50;

unsigned long previousMillis = 0;
const long BLINK_INTERVAL = 1000;   // interval at which to blink LED (milliseconds)
int ledState = LOW;
float min_dist = 999;
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
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial);
  mySerial.begin(115200);

  mySerial.println("Witoj Michał");

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float metry = distance / 1000;
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    //mySerial.println("Start bit: ");
    //mySerial.println(startBit);

    if (angle > 315 or angle < 45)
    {
      if (metry < min_dist and metry != 0)
      {
        min_dist = metry;
      }
    }


    if (startBit == 1)
    {
      //mySerial.println("min dist: ");
      mySerial.println(min_dist);

      if (min_dist <= ODL_1 )
      {


        if (currentMillis - previousMillis >= BLINK_INTERVAL) {
          // if the LED is off turn it on and vice-versa:
          if (ledState == LOW)
          {
            ledState = HIGH;
          }
          else {
            ledState = LOW;
          }

          // set the LED with the ledState of the variable:
          mySerial.println(ledState);

          // save the last time you blinked the LED
          previousMillis = currentMillis;
        }
      }
      else if (min_dist > ODL_1 and min_dist <= ODL_2 )
      {
        zakres_2++;
        if (zakres_2 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("2");
          zakres_2 = 0;
        }
      }
      else if (min_dist > ODL_2  and min_dist <= ODL_3 )
      {
        zakres_3++;
        if (zakres_3 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("3");
          zakres_3 = 0;
        }
      }
      else if (min_dist > ODL_3 and min_dist <= ODL_4 )
      {
        zakres_4++;
        if (zakres_4 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("4");
          zakres_4 = 0;
        }
      }
      else if (min_dist > ODL_4 and min_dist <= ODL_5 )
      {
        zakres_5++;
        if (zakres_5 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("5");
          zakres_5 = 0;
        }
      }
      else if (min_dist > ODL_5 and min_dist <= ODL_6 )
      {
        zakres_6++;
        if (zakres_6 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("6");
          zakres_6 = 0;
        }
      }
      else if (min_dist > ODL_6 and min_dist <= ODL_7 )
      {
        zakres_7++;
        if (zakres_7 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("7");
          zakres_7 = 0;
        }
      }
      else if (min_dist > ODL_7 and min_dist <= ODL_8 )
      {
        zakres_8++;
        if (zakres_8 >= liczba_skanow)
        {
          dopisz();
          mySerial.println("8");
          zakres_8 = 0;
          {
          }

        }
      }
      min_dist = 999;
    }

  }
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
}
