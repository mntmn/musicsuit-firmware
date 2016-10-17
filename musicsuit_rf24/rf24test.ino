#include <SPI.h>
#include "RF24.h"

bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);

byte addresses[][6] = {"1Node","2Node"};

char inbuf[32];

void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24"));
  
  radio.begin();

  radio.setPALevel(RF24_PA_LOW);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  
  radio.startListening();
}

void loop() {
  radio.read(inbuf, 32);
  Serial.print(inbuf);
}

