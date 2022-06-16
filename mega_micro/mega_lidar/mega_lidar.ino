

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
//W bibliotece arduino w pliku
#include <RPLidar.h>

// You need to create an driver instance
RPLidar lidar;

#define NUM_LEDS 12
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal
float min_dist = 10000;
float diody_metry = 10000;



int CZULOSC = 0;  //USTAWIENIE CZUŁOŚCI
int liczba_skanow = 2;
byte LED_SWIEC = 0;  //0 nie swiec, 99 migaj
const float ODL_MIN = 2;
const float SKOK = 0.3;
float ZAKRESY[NUM_LEDS];


void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  for (int i = 0; i < NUM_LEDS; i++)
  {
    ZAKRESY[i] = ODL_MIN + (i * SKOK);
  }
  lidar.begin(Serial1);
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 255);
  delay(1000);
  analogWrite(RPLIDAR_MOTOR, 0);
}

void wyslijDiody();

float printL = 0;
float printP = 0;
byte printQ = 0;

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    

    //if (angle > 355 or angle < 8)
    if (angle > 280 or angle < 80)
    {
      float metry = distance / 1000;
      double rad = angle * DEG_TO_RAD;
      double l_lewo = abs(0.25 / sin(rad)); //(patrząc od silnika) bez minusa 
      double l_prawo = abs(-0.25 / sin(rad)); //(patrząc od silnika) minus
      if(metry <= l_lewo or metry <= l_prawo)
      {
        //Serial.println(quality);
        //Serial.println(l_lewo);
        //Serial.println(l_prawo);
        if (metry < min_dist and metry > 0 and metry < 10000)
        {
          min_dist = metry;
          
          printL = l_lewo;
          printP = l_prawo;
          printQ = quality;
        }
      }
    }



    //if (startBit == true and min_dist < 20)
    if (startBit == true and min_dist < 20)
    {
      Serial1.flush();
      liczba_skanow++;

      if (liczba_skanow >= CZULOSC)
      {
        liczba_skanow = 0;
        

        diody_metry = min_dist;
        
        //sprawdzenie czy nie było pomiaru z dupy

        wyslijDiody();
        
        min_dist = 10000;

        
        Serial.println(diody_metry);
        //Serial.println(printL);
        //Serial.println(printP);
        //Serial.println(printQ);
        Serial.println("");
        


      }

    }


  } else {
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


void wyslijDiody()
{
  LED_SWIEC = 0;
  if (diody_metry < ZAKRESY[0])
  {
    LED_SWIEC = 99;
  }
  else if (diody_metry > ZAKRESY[(NUM_LEDS - 1)])
  {
    LED_SWIEC = 0;
  }
  else
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (diody_metry >= ZAKRESY[i])
      {
        LED_SWIEC = NUM_LEDS - (i);
      }

    }
  }


  Serial.println(LED_SWIEC);
  Serial2.write(LED_SWIEC);


}
