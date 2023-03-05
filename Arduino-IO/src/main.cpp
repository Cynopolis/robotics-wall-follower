#include <Arduino.h>
#include "Pinout.h"
#include "SerialMessage.h"
#include "Sonar.h"

Sonar sonar(trig_pin, echo_pin);
Servo servo;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

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
  EncoderDiffDrive wheels(leftMotor, rightMotor, 60); //TODO: Change this to the actual wheel separation
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
  wheels.setPID(2,0.01,0.5);
  // attach the interrupts
  attachInterrupt(digitalPinToInterrupt(left_encoder_pinA), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pinA), rightEncoderInc, CHANGE);

  servo.attach(servo_pin);
  servo.write(90);
  sonar.attachServo(servo);

  sonar.enableScanMode(false);
  Serial.println("Started");
  timer = millis();

}

// TODO: Finish writing this function
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case MOTOR_READ:
      Serial.print("!MTR,");
      #ifdef USE_ENCODERS
        // print the pose
        Pose pose = *(wheels.getCurrentPose());
        Serial.print(String(pose.x) + "," + String(pose.y) + "," + String(pose.theta) + "," + String(pose.d_x) + "," + String(pose.d_y) + "," + String(pose.d_theta));
      #else
        Serial.print(String(wheels.getAcceleration()) + "," + String(wheels.getMaxVelocity()));
        Serial.print("," + String(wheels.getLeftTargetVelocity()) + "," + String(wheels.getRightTargetVelocity()));
      #endif
      Serial.println(";");
      break;
    case MOTOR_WRITE:
      if(args_length < 3) break;
      Serial.print("!MTR_WRT,");
      Serial.print(String(args[1]) + "," + String(args[2]) + "," + String(args[3]) + "," + String(args[4]) + "," + String(args[5]) + "," + String(args[6]));
      Serial.println(";");
      // set the new target pose
      Pose targetPose = *(wheels.getTargetPose());
      targetPose.x = args[1];
      targetPose.y = args[2];
      targetPose.theta = args[3];
      targetPose.d_x = args[4];
      targetPose.d_y = args[5];
      targetPose.d_theta = args[6];
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
      Serial.print("SNR,");
      Serial.print(args[1]==1);
      Serial.print(",");
      Serial.print(args[2]);
      Serial.println(";");
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

  #ifdef USE_ENCODERS
    wheels.update(leftEncoderCount, rightEncoderCount);
  #else
    wheels.update();
  #endif
  sonar.update();
}