#include <Arduino.h>
#include "Pinout.h"
#include "SerialMessage.h"
#include "Motor.h"

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
SerialMessage ser((int)115200);

unsigned long timer = 0;
void setup() {
  timer = millis();
}

// TODO: Finish writing this function
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case MOTOR_READ:
      Serial.print("!MTR,");
      Serial.print("," + String(wheels.getAcceleration()) + "," + String(wheels.getMaxVelocity()));
      Serial.print("," + String(wheels.getLeftTargetVelocity()) + "," + String(wheels.getRightTargetVelocity()));
      #ifdef USE_ENCODERS
        Serial.print("," + String(wheels.getDistance()) + "," + String(wheels.getAngle()));
      #endif
      Serial.println(";");
      break;
    case MOTOR_WRITE:
      Serial.println("MTR");
      #ifdef USE_ENCODERS
        wheels.setCurrentPosition(double(args[1])/1000, double(args[2])/1000);
        wheels.setTarget(double(args[3])/1000, double(args[4])/1000);
      #else
        wheels.setDirectionVector(args[1], args[2]);
      #endif
      break;
    case SONAR_READ:
      //ser.println("SONAR_READ");
      break;
    case SONAR_WRITE:
      Serial.println("SNR");
      break;
    case IR_READ:
      //ser.println("IR_READ");
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
    int args_length = ser.getArgsLength();
    doSerialCommand(args, args_length);
    ser.clearNewData();
    
  }
}