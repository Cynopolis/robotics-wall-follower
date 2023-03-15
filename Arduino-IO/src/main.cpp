#include <Arduino.h>
#include "Pinout.h"
#include "SerialMessage.h"
#include "Sonar.h"
#include "EncoderDiffDrive.h"


Sonar sonar(trig_pin, echo_pin);
Servo servo;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Incriment / Decrement depending on encoder state during an interrupt
void leftEncoderInc(){
  if (digitalRead(left_encoder_pinA) == digitalRead(left_encoder_pinB)) {
    leftEncoderCount++;
    return;
  }
  leftEncoderCount--;
  
}

// Incriment / Decrement depending on encoder state during an interrupt
void rightEncoderInc(){
  if (digitalRead(right_encoder_pinA) == digitalRead(right_encoder_pinB)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

EncodedMotor leftMotor(pinLF, pinLB, Lpwm_pin, left_encoder_pinA, left_encoder_pinB);
EncodedMotor rightMotor(pinRF, pinRB, Rpwm_pin, right_encoder_pinA, right_encoder_pinB);
EncoderDiffDrive wheels(leftMotor, rightMotor, 165); //TODO: Change this to the actual wheel separation

// Object to handle serial communication
SerialMessage ser;

void setup() {
  Serial.begin(serial_baud);
  Serial.println("Starting up...");


  wheels.setup();
  wheels.setPID(0.1,0.00,0.00);
  // attach the interrupts
  attachInterrupt(digitalPinToInterrupt(left_encoder_pinA), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pinA), rightEncoderInc, CHANGE);

  servo.attach(servo_pin);
  servo.write(90);
  sonar.attachServo(servo);

  sonar.enableScanMode(false);
  Serial.println("Started");
}

// TODO: Finish writing this function
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case MOTOR_READ:{
      Serial.print("!MTR,");
      #ifdef USE_ENCODERS
        // print the current pose
        Pose *pose = (wheels.getCurrentPose());
        for(auto i = 0; i < 6; i++) {
          Serial.print(pose->get(i),4);
          Serial.print(",");
        }
        // print the target pose
        pose = (wheels.getTargetPose());
        for(auto i = 0; i < 3; i++) {
          Serial.print(pose->get(i),4);
          Serial.print(",");
        }

      #else
        Serial.print(String(wheels.getAcceleration()) + "," + String(wheels.getMaxVelocity()));
        Serial.print("," + String(wheels.getLeftTargetVelocity()) + "," + String(wheels.getRightTargetVelocity()));
      #endif
      Serial.println(";");
      break;
    }
    case MOTOR_WRITE:{
      if(args_length < 3) break;
      Serial.print("!MTR_WRT,");
      for(int i = 1; i < args_length; i++) {
        Serial.print(args[i]);
        Serial.print(",");
      }
      Serial.println(";");
      // set the new target pose
      Pose *targetPose = (wheels.getTargetPose());
      targetPose->x = args[1];
      targetPose->y = args[2];
      targetPose->theta = args[3];
      wheels.print();
      break;
    }
    case SONAR_READ:{ 
      Serial.print("!SNR,");
      Serial.print(sonar.getAngleIncrement());
      sonar.print();
      Serial.println(";");
      break;
    }
    case SONAR_WRITE:{
      if(args_length < 2) break;
      sonar.enableScanMode(args[1]==1);
      sonar.setAngleIncrement(args[2]);
      Serial.print("SNR,");
      Serial.print(args[1]==1);
      Serial.print(",");
      Serial.print(args[2]);
      Serial.println(";");
      break;
    }
    case IR_READ:{
      Serial.println("IR_READ");
      break;
    }
    default:{
      Serial.println("ERR");
      break;
    }
  }
}

unsigned long timer = 0;

void loop() {
  ser.update();
  if (ser.isNewData()) {
    int * args = ser.getArgs();
    int args_length = ser.getPopulatedArgs();

    // ser.printArgs();

    doSerialCommand(args, args_length);
    ser.clearNewData();
  }

  #ifdef USE_ENCODERS
    wheels.update(leftEncoderCount, rightEncoderCount);
  #else
    wheels.update();
  #endif
  sonar.update();

  if(millis() - timer > 5000){
    //Serial.println("Still connected");
    timer = millis();
  }
}