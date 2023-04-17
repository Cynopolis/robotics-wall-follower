#include <Arduino.h>
#include "PINOUT.h"
#include "SerialMessage.h"
#include "DiffDrive.h"
#include "BluetoothSerialMessage.h"
#include "IMU.h"


// Sonar sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
//Servo servo;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// Incriment / Decrement depending on encoder state during an interrupt
void leftEncoderInc(){
  if (digitalRead(LEFT_ENC_A_PIN) == digitalRead(LEFT_ENC_B_PIN)) {
    leftEncoderCount++;
    return;
  }
  leftEncoderCount--;
  
}

// Incriment / Decrement depending on encoder state during an interrupt
void rightEncoderInc(){
  if (digitalRead(RIGHT_ENC_A_PIN) == digitalRead(RIGHT_ENC_B_PIN)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

SpeedMotor leftMotor(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACK_PIN, LEFT_MOTOR_PWM_PIN, 0,  &leftEncoderCount);
SpeedMotor rightMotor(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACK_PIN, RIGHT_MOTOR_PWM_PIN, 1, &rightEncoderCount);
DiffDrive wheels(&leftMotor, &rightMotor, 125); //151/1.86
IMU imu;

// Object to handle serial communication
SerialMessage ser;
BluetoothSerial bleSerial;
BluetoothSerialMessage bleSer(&bleSerial);

void setup() {
  Serial.begin(115200);
  bleSer.init();
  delay(1000);
  imu.begin();

  Serial.println("Starting up...");
  bleSerial.println("Starting up...");

  wheels.begin();
  wheels.setVelocityPID(1, 0, 0);
  // wheels.setPID(0, 0, 0);
  // // attach the interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderInc, CHANGE);
  
  // servo.attach(servo_pin);
  // servo.write(90);
  // sonar.attachServo(servo);
  
  // sonar.enableScanMode(false);
  Serial.println("Started");
  bleSerial.println("Started");
}

// takes the serial args and executes the command
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case ERR:{
      Serial.println("!ERR;");
      bleSerial.println("!ERR;");
      break;
    }
    case GET_POSITION:{
      Serial.print("!MTR,");
      bleSerial.print("!MTR,");
      // print the current pose
      xyzData pose = (wheels.getCurrentPose());
      for(auto i = 0; i < 3; i++) {
        Serial.print(pose.get(i),4);
        bleSerial.print(pose.get(i),4);
        Serial.print(",");
        bleSerial.print(",");
      }
      // print the target pose
      pose = (wheels.getTargetPose());
      for(auto i = 0; i < 3; i++) {
        Serial.print(pose.get(i),4);
        bleSerial.print(pose.get(i),4);
        Serial.print(",");
        bleSerial.print(",");
      }

      Serial.println(";");
      bleSerial.println(";");
      break;
    }
    // set the current pose
    case SET_POSITION:{
      if(args_length < 4) break;
      Serial.print("!MTR_WRT,");
      bleSerial.print("!MTR_WRT,");
      for(int i = 1; i < args_length; i++) {
        Serial.print(args[i]);
        bleSerial.print(args[i]);
        Serial.print(",");
        bleSerial.print(",");
      }
      Serial.println(";");
      bleSerial.println(";");
      // set the new target pose
      wheels.setCurrentPose(args[1], args[2], float(args[3])*PI/180);
      //wheels.print();
      break;
    }
    // set the drive train control constants
    case SET_VELOCITY_CONSTANTS:{
      if(args_length < 4) break;
      Serial.print("!VELWRT,");
      bleSerial.print("!VELWRT,");
      for(int i = 1; i < args_length; i++) {
        Serial.print(float(args[i])/1000.0);
        bleSerial.print(float(args[i])/1000.0);
        Serial.print(",");
        bleSerial.print(",");
      }
      Serial.println(";");
      bleSerial.println(";");
      wheels.setVelocityPID(float(args[1])/1000.0, float(args[2])/1000.0, float(args[3])/1000.0);
      break;
    }
    case SET_ANGLE_CONSTANTS:{
      if(args_length < 4) break;
      Serial.print("!ANGLEWRT,");
      bleSerial.print("!ANGLEWRT,");
      for(int i = 1; i < args_length; i++) {
        Serial.print(float(args[i])/1000.0);
        bleSerial.print(float(args[i])/1000.0);
        Serial.print(",");
        bleSerial.print(",");
      }
      Serial.println(";");
      bleSerial.println(";");
      wheels.setVelocityPID(float(args[1])/1000.0, float(args[2])/1000.0, float(args[3])/1000.0);
      break;
    }
    // set the motor pid constants
    case SET_MOTOR_PID:{
      if(args_length < 5) break;
      Serial.print("!MOTORPIDWRT,");
      bleSerial.print("!MOTORPIDWRT,");
      for(int i = 1; i < args_length; i++) {
        Serial.print(float(args[i])/1000.0);
        bleSerial.print(float(args[i])/1000.0);
        Serial.print(",");
        bleSerial.print(",");
      }
      Serial.println(";");
      bleSerial.println(";");
      if(args[1] == 0) {
        leftMotor.setPID(float(args[2])/1000.0, float(args[3])/1000.0, float(args[4])/1000.0);
      } else {
        rightMotor.setPID(float(args[2])/1000.0, float(args[3])/1000.0, float(args[4])/1000.0);
      }
      break;
    }
    default:{
      Serial.println("!ERR;");
      bleSerial.println("!ERR;");
      break;
    }
    // case 3:{
    //   if(args_length < 4) break;
    //   Serial.print("!INCWRT,");
    //   bleSerial.print("!INCWRT,");
    //   for(int i = 1; i < args_length; i++) {
    //     Serial.print(float(args[i]));
    //     bleSerial.print(float(args[i]));
    //     Serial.print(",");
    //     bleSerial.print(",");
    //   }
    //   Serial.println(";");
    //   xyzData currentPose = wheels.getCurrentPose();
    //   wheels.setTargetPose(currentPose.x + args[1], currentPose.y + args[2], currentPose.z + float(args[3])*PI/180);
  }
}

unsigned long timer = 0;

void loop() {
  ser.update();
  bleSer.update();
  if (ser.isNewData()) {
    int * args = ser.getArgs();
    int args_length = ser.getPopulatedArgs();

    doSerialCommand(args, args_length);
    ser.clearNewData();
  }

  if(bleSer.isNewData()) {
    int * args = bleSer.getArgs();
    int args_length = bleSer.getPopulatedArgs();

    doSerialCommand(args, args_length);
    bleSer.clearNewData();
  }
  imu.update();
  wheels.update(&imu);
  // wheels.update();
  // if(millis() - timer > 1000){
  //   imu.print();
  //   timer = millis();
  // }
}
