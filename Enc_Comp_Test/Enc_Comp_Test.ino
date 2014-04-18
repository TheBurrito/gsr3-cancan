#include <Encoder.h>
#include <Pins.h>

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

void setup() {
}

void loop() {
  long encL = m1Enc.read();
  long encR = m2Enc.read();
  delay(10);
}
