#ifndef PINOUT
#define PINOUT

// define motor pins
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
#define pinLB 9 // will make the left motor go backwards
#define pinLF 4 // will make the left motor go forwards
#define pinRB 7 // will make the right motor go backwards
#define pinRF 8 // will make the right motor go forwards

//define encoder pins
#define left_encoder_pinA 2
#define left_encoder_pinB 10
#define right_encoder_pinA 3
#define right_encoder_pinB 11

// define the pins for the sonar sensor
#define echo_pin A0
#define trig_pin A1

// define the pin for the servo
#define servo_pin A2

// define the pin for the IR sensor
#define IR_pin 12

//Define pins for I2C
#define SDA_pin 18
#define SCL_pin 19
#endif