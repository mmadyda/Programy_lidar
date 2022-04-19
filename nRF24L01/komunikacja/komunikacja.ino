

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

const unsigned int pipe = 519;

RF24 radio(CE_PIN, CSN_PIN);
int dane[2];
int msg[2];

void setup() {
  dane[0] = 90;
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipe);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  radio.write(dane, sizeof(dane));

}

void loop() {
  radio.write(dane, sizeof(dane));
  radio.read(msg, sizeof(msg));
  Serial.println(msg[0]);
  delay(100);
} // loop
