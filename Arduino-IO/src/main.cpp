#include <Arduino.h>
#include "Pinout.h"
#include "SerialMessage.h"
#include "Motor.h"
#include "Sonar.h"

Sonar sonar(trig_pin, echo_pin);
Servo servo;

Motor leftMotor(pinLF, pinLB, Lpwm_pin);
Motor rightMotor(pinRF, pinRB, Rpwm_pin);

#ifdef USE_ENCODERS
  #include "DiffDriveFeedback.h"
  Encoder leftEncoder(left_encoder_pinA, left_encoder_pinB);
  Encoder rightEncoder(right_encoder_pinA, right_encoder_pinB);
  DiffDriveFeedback wheels(leftMotor, rightMotor, leftEncoder, rightEncoder);
#else
  #include "DiffDrive.h"
  DiffDrive wheels(leftMotor, rightMotor);
#endif
SerialMessage ser;

unsigned long timer = 0;
void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  servo.attach(servo_pin);
  servo.write(90);
  sonar.attachServo(servo);

  sonar.enableScanMode(true);
  Serial.println("Started");
  timer = millis();
}

// TODO: Finish writing this function
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case MOTOR_READ:
      Serial.print("!MTR,");
      Serial.print(String(wheels.getAcceleration()) + "," + String(wheels.getMaxVelocity()));
      Serial.print("," + String(wheels.getLeftTargetVelocity()) + "," + String(wheels.getRightTargetVelocity()));
      #ifdef USE_ENCODERS
        Serial.print("," + String(wheels.getDistance()) + "," + String(wheels.getAngle()));
      #endif
      Serial.println(";");
      break;
    case MOTOR_WRITE:
      #ifdef USE_ENCODERS
        if(args_length < 5) break;
        Serial.println("MTR");
        wheels.setCurrentPosition(double(args[1])/1000, double(args[2])/1000);
        wheels.setTarget(double(args[3])/1000, double(args[4])/1000);
      #else
        if(args_length < 3) break;
        Serial.println("MTR");
        wheels.setDirectionVector(args[1], args[2]);
      #endif
      break;
    case SONAR_READ:
      Serial.print("!SNR,");
      Serial.print(sonar.getAngleIncrement());
      sonar.print();
      Serial.println(";");
      break;
    case SONAR_WRITE:
      if(args_length < 2) break;
      sonar.enableScanMode(args[1]==1);
      sonar.setAngleIncrement(args[2]);
      Serial.println("SNR");
      break;
    case IR_READ:
      Serial.println("IR_READ");
      break;
    default:
      Serial.println("ERR");
      break;
  }
}

void loop() {
  ser.update();
  if (ser.isNewData()) {
    int * args = ser.getArgs();
    int args_length = ser.getPopulatedArgs();

    ser.printArgs();

    doSerialCommand(args, args_length);
    ser.clearNewData();
  }

  if(millis()-timer > 1000){
    timer = millis();
    digitalWrite(13, !digitalRead(13));
  }
  wheels.update();
  sonar.update();
}