#include <Arduino.h>
#include "Pinout.h"
#include "SerialMessage.h"

volatile int8_t leftEncoderCount = 0;
volatile int8_t rightEncoderCount = 0;

// Incriment / Decrement depending on encoder state during an interrupt
void leftEncoderInc(){
  if (digitalRead(left_encoder_pinA) == digitalRead(left_encoder_pinB)) {
    leftEncoderCount--;
    return;
  }
  leftEncoderCount++;
  
}

// Incriment / Decrement depending on encoder state during an interrupt
void rightEncoderInc(){
  if (digitalRead(right_encoder_pinA) == digitalRead(right_encoder_pinB)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

#ifdef USE_ENCODERS
  #include "EncoderDiffDrive.h"
  EncodedMotor leftMotor(pinLF, pinLB, Lpwm_pin, left_encoder_pinA, left_encoder_pinB);
  EncodedMotor rightMotor(pinRF, pinRB, Rpwm_pin, right_encoder_pinA, right_encoder_pinB);
  EncoderDiffDrive wheels(leftMotor, rightMotor);
#else
  #include "DiffDrive.h"
  Motor leftMotor(pinLF, pinLB, Lpwm_pin);
  Motor rightMotor(pinRF, pinRB, Rpwm_pin);
  DiffDrive wheels(leftMotor, rightMotor);
#endif
SerialMessage ser;

unsigned long timer = 0;
void setup() {
  Serial.begin(serial_baud);
  timer = millis();
  Serial.println("Starting up...");
  pinMode(debug_pin, OUTPUT);

  // digitalWrite(13, HIGH);
  // delay(100);
  // digitalWrite(13, LOW);
  // this must be called before we attach any interrupts
  wheels.setup();
  wheels.setPID(7, 0.005, 0.01);
  // attach the interrupts
  attachInterrupt(digitalPinToInterrupt(left_encoder_pinA), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pinA), rightEncoderInc, CHANGE);

}

// TODO: Finish writing this function
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case MOTOR_READ:
      Serial.print("!MTR,");
      // Serial.print(String(wheels.getAcceleration()) + "," + String(wheels.getMaxVelocity()));
      // Serial.print("," + String(wheels.getLeftTargetVelocity()) + "," + String(wheels.getRightTargetVelocity()));
      #ifdef USE_ENCODERS
        //Serial.print("," + String(wheels.getLeftDistance()) + "," + String(wheels.getRightDistance()));
      #endif
      Serial.println(";");
      break;
    case MOTOR_WRITE:
      if(args_length < 3) break;
      Serial.println("MTR");
      //wheels.setTargetVelocity(args[1], args[2]);
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
  // ser.update();
  // if (ser.isNewData()) {
  //   int * args = ser.getArgs();
  //   int args_length = ser.getPopulatedArgs();

  //   //ser.printArgs();

  //   doSerialCommand(args, args_length);
  //   ser.clearNewData();
  // }

  // if(millis()-timer > 1000){
  //   timer = millis();
  //   digitalWrite(13, !digitalRead(13));
  // }
  #ifdef USE_ENCODERS
    wheels.update(leftEncoderCount, rightEncoderCount);
    //delay(10);
  #else
    wheels.update();
  #endif
}