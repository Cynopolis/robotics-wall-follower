#include <Arduino.h>
#include "Pinout.h"
#include "DiffDrive.h"

Motor leftMotor(pinLF, pinLB, Lpwm_pin);
Motor rightMotor(pinRF, pinRB, Rpwm_pin);
DiffDrive wheels(leftMotor, rightMotor);

unsigned long timer = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("Will begin testing motors in 3 seconds");
  delay(3000);
  timer = millis();
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
  wheels.setDirectionVector(50, angle);
  wheels.update();

}