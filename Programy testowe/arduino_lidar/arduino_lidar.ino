/*
   Ctrl+T auto format kodu
   ---------------------------------
   1. Download this sketch code to your Arduino board
   2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
   3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
*/


#include <RPLidar.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 4); // RX, TX LIDAR

RPLidar lidar;

#define RPLIDAR_MOTOR 2 // The PWM pin for control the speed of RPLIDAR's motor.
#define LED_1 6
#define LED_2 7
#define LED_3 8
#define LED_4 9
#define LED_5 10
#define LED_6 11
#define LED_7 12
#define LED_8 13

float ODL_1 = 0.6;
float ODL_2 = 0.9;
float ODL_3 = 1.2;
float ODL_4 = 1.5;
float ODL_5 = 1.8;
float ODL_6 = 2.1;
float ODL_7 = 2.4;
float ODL_8 = 2.7;

int CZULOSC = 1;  //USTAWIENIE CZUŁOŚCI
int liczba_skanow = 0;

unsigned long previousMillis = 0;
const long BLINK_INTERVAL = 500;   // interval at which to blink LED (milliseconds)
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


  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
  pinMode(LED_7, OUTPUT);
  pinMode(LED_8, OUTPUT);

  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  digitalWrite(LED_4, HIGH);
  digitalWrite(LED_5, HIGH);
  digitalWrite(LED_6, HIGH);
  digitalWrite(LED_7, HIGH);
  digitalWrite(LED_8, HIGH);
}

void loop() {
  unsigned long currentMillis = millis();
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float metry = distance / 1000;
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    //if (angle > 270 or angle < 90)

    if (angle > 270 or angle < 90)
    {
      if (metry < min_dist and metry != 0)
      {
        min_dist = metry;
      }
    }


    if (startBit == 1)
    {

      liczba_skanow++;
      dopisz();
      if (liczba_skanow >= CZULOSC)
      {

        liczba_skanow = 0;



        if (min_dist <= ODL_1 )
        {
          if (currentMillis - previousMillis >= BLINK_INTERVAL) {
            if (ledState == LOW)
            {
              ledState = HIGH;
              digitalWrite(LED_1, ledState);
              digitalWrite(LED_2, ledState);
              digitalWrite(LED_3, ledState);
              digitalWrite(LED_4, ledState);
              digitalWrite(LED_5, ledState);
              digitalWrite(LED_6, ledState);
              digitalWrite(LED_7, ledState);
              digitalWrite(LED_8, ledState);
            }
            else {
              ledState = LOW;
              digitalWrite(LED_1, ledState);
              digitalWrite(LED_2, ledState);
              digitalWrite(LED_3, ledState);
              digitalWrite(LED_4, ledState);
              digitalWrite(LED_5, ledState);
              digitalWrite(LED_6, ledState);
              digitalWrite(LED_7, ledState);
              digitalWrite(LED_8, ledState);
            }

            previousMillis = currentMillis;
          }
        }
        else if (min_dist > ODL_1 and min_dist <= ODL_2 )
        {
          mySerial.println("2");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, LOW);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, HIGH);

        }
        else if (min_dist > ODL_2  and min_dist <= ODL_3 )
        {
          mySerial.println("3");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, LOW);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, HIGH);

        }
        else if (min_dist > ODL_3 and min_dist <= ODL_4 )
        {
          mySerial.println("4");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, LOW);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, HIGH);
        }
        else if (min_dist > ODL_4 and min_dist <= ODL_5 )
        {
          mySerial.println("5");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, LOW);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, HIGH);
        }
        else if (min_dist > ODL_5 and min_dist <= ODL_6 )
        {
          mySerial.println("6");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, LOW);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, HIGH);
        }
        else if (min_dist > ODL_6 and min_dist <= ODL_7 )
        {
          mySerial.println("7");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, LOW);
          digitalWrite(LED_8, HIGH);
        }
        else if (min_dist > ODL_7 and min_dist <= ODL_8 )
        {
          mySerial.println("8");
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, LOW);

        }
        else
        {
          digitalWrite(LED_1, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_4, HIGH);
          digitalWrite(LED_5, HIGH);
          digitalWrite(LED_6, HIGH);
          digitalWrite(LED_7, HIGH);
          digitalWrite(LED_8, HIGH);
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
}
