#include <Arduino.h>
#include "Pinout.h"

long leftEncoderCount = 0;
long rightEncoderCount = 0;

void leftEncoderInc(){
  if (digitalRead(left_encoder_pinA) && digitalRead(left_encoder_pinB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderInc(){
  if (digitalRead(right_encoder_pinA) && digitalRead(right_encoder_pinB)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

unsigned long timer = 0;
void setup() {
  Serial.begin(serial_baud);
  // TODO: set up the encoded wheels and make sure hteir initialized first
  attachInterrupt(digitalPinToInterrupt(left_encoder_pinA), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pinA), rightEncoderInc, CHANGE);

}

int angle = 0;

void loop() {
  if (millis() - timer > 2000) {
    angle += 45;
    Serial.print("Angle: ");
    Serial.println(angle);
    if (angle > 360) {
      angle = 0;
    }
    timer = millis();
  }
}