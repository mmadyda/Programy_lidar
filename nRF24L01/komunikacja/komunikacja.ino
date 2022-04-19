

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

const unsigned int pipe = 519;
const unsigned int pipeRead = 500;
const unsigned int pipeWrite = 800;


RF24 radio(CE_PIN, CSN_PIN);
int dane[2];
int msg[2];

void setup() {
  dane[0] = 90;
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipeWrite);
  radio.openReadingPipe(1, pipeRead);
  //adio.startListening();
  //radio.write(dane, sizeof(dane));

}

void loop() {
  radio.stopListening();
  radio.write(dane, sizeof(dane));
  radio.startListening();
  radio.read(msg, sizeof(msg));
  radio.stopListening();
  Serial.println(msg[0]);
  delay(100);
} // loop
