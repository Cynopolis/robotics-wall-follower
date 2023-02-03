#include <Arduino.h>
#include "Pinout.h"
#include "EncoderDiffDrive.h"

long leftEncoderCount = 0;
long rightEncoderCount = 0;

// Incriment / Decrement depending on encoder state during an interrupt
void leftEncoderInc(){
  if (digitalRead(left_encoder_pinA) && digitalRead(left_encoder_pinB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

// Incriment / Decrement depending on encoder state during an interrupt
void rightEncoderInc(){
  if (digitalRead(right_encoder_pinA) && digitalRead(right_encoder_pinB)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

EncodedMotor leftMotor(pinLF, pinLB, Lpwm_pin, left_encoder_pinA, left_encoder_pinB);
EncodedMotor rightMotor(pinRF, pinRB, Rpwm_pin, right_encoder_pinA, right_encoder_pinB);
EncoderDiffDrive diffDrive(leftMotor, rightMotor);

unsigned long timer = 0;
void setup() {
  Serial.begin(serial_baud);
  // this must be called before we attach any interrupts
  diffDrive.setup();
  // attach the interrupts
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

  diffDrive.update(&leftEncoderCount, &rightEncoderCount);
}