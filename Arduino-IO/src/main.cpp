#include <Arduino.h>
#include "Pinout.h"
#include "MemorySize.h"
#include "DiffDrive.h"
#include "SerialMessage.h"

Motor leftMotor(pinLF, pinLB, Lpwm_pin);
Motor rightMotor(pinRF, pinRB, Rpwm_pin);
DiffDrive wheels(leftMotor, rightMotor);
SerialMessage ser(115200);

unsigned long timer = 0;
void setup() {
  Serial.println("Will begin testing motors in 3 seconds");
  delay(3000);
  timer = millis();
}

int angle = 0;

void loop() {

}