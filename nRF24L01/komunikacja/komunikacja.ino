

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

//w słupkach zamieniamy adresy pipeRead z pipeWrite
const unsigned int pipeRead = 500;
const unsigned int pipeWrite = 800;


RF24 radio(CE_PIN, CSN_PIN);
int dane[2];
int msg[2];

void setup() {
  dane[0] = 30;
  Serial.begin(9600);
  
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_HIGH);           //ustaw wzmocnienie modułu radiowego na wysokie 
  radio.openWritingPipe(pipeWrite);
  radio.openReadingPipe(1, pipeRead);
  //adio.startListening();
  //radio.write(dane, sizeof(dane));

}

void loop() {
  radio.stopListening();
  radio.write(dane, sizeof(dane));
  radio.startListening();
  while(!radio.available());
  if(radio.available())
  {
  radio.read(msg, sizeof(msg));
  Serial.println(msg[0]);
  }
  
  
  delay(100);
  
} // loop
