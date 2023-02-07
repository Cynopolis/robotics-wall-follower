#include <Arduino.h>
#include "Pinout.h"
#include "SerialMessage.h"
#include "Sonar.h"

Sonar sonar(trig_pin, echo_pin);
Servo servo;

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
  wheels.setPID(1.7,0.4,1.0);
  // attach the interrupts
  attachInterrupt(digitalPinToInterrupt(left_encoder_pinA), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pinA), rightEncoderInc, CHANGE);

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
      #ifdef USE_ENCODERS
        Serial.print(String(wheels.getLeftDistance()) + "," + String(wheels.getRightDistance()));
      #else
        Serial.print(String(wheels.getAcceleration()) + "," + String(wheels.getMaxVelocity()));
        Serial.print("," + String(wheels.getLeftTargetVelocity()) + "," + String(wheels.getRightTargetVelocity()));
      #endif
      Serial.println(";");
      break;
    case MOTOR_WRITE:
      if(args_length < 3) break;
      Serial.print("!MTR_WRT,");
      Serial.print(args[1]);
      Serial.print(",");
      Serial.print(args[2]);
      wheels.setDistances(args[1], args[2]);
      // optionally allow resetting the distances of the wheels
      #ifdef USE_ENCODERS
        if (args_length >= 5){
          Serial.print(args[3]);
          Serial.print(",");
          Serial.print(args[4]);
          wheels.setDistances(args[3], args[4]);
        }
      #endif
      Serial.println(";");
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

    //ser.printArgs();

    doSerialCommand(args, args_length);
    ser.clearNewData();
  }

  // if(millis()-timer > 1000){
  //   timer = millis();
  //   digitalWrite(13, !digitalRead(13));
  // }
  #ifdef USE_ENCODERS
    wheels.update(leftEncoderCount, rightEncoderCount);
    //delay(10);
  #else
    wheels.update();
  sonar.update();
  #endif
}