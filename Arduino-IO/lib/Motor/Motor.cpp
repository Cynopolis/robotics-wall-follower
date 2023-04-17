#include "Motor.h"

Motor::Motor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t pwmChannel) : 
forwardPin(forwardPin), backwardPin(backwardPin), pwmPin(pwmPin), pwmChannel(pwmChannel){}

Motor::Motor(uint8_t pwmPin, uint8_t pwmChannel) : forwardPin(-1), backwardPin(-1), pwmPin(pwmPin), pwmChannel(pwmChannel){}

float Motor::getVelocity(){
    return this->currentVelocity;
}

void Motor::begin(){  
    if(forwardPin != 255 && backwardPin != 255){
        pinMode(forwardPin, OUTPUT);
        pinMode(backwardPin, OUTPUT);
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, LOW);
        ledcAttachPin(pwmPin, pwmChannel);
        ledcSetup(pwmChannel, 24000, 8);
        Serial.print("Motor::begin() pwmChannel: ");
        Serial.print(pwmChannel);
        Serial.print(" pwmPin: ");
        Serial.print(pwmPin);
        Serial.print(" forwardPin: ");
        Serial.print(forwardPin);
        Serial.print(" backwardPin: ");
        Serial.println(backwardPin);
    }
    else{
        Serial.print("Motor::begin() Servo mode on pin:");
        Serial.println(pwmPin);
        servo.attach(pwmPin, pwmChannel, 0, 180, 670, 2330);
    }
}

float Motor::update(){
    return currentVelocity;
}

void Motor::setVelocity(int velocity){
    if(forwardPin != 255 && backwardPin != 255){
        Serial.println("Wrong function ran!");
        setPWMVelocity(velocity);
    } else {
        setServoVelocity(velocity);
    }
}

void Motor::setServoVelocity(int velocity){
    // make sure the velocity is in the range of -255 to 255
    if(velocity > 255){
        velocity = 255;
    } else if(velocity < -255){
        velocity = -255;
    }

    currentVelocity = (float)velocity;
    Serial.print("Velocity: ");
    Serial.println(velocity);
    servo.write(map(velocity, -255, 255, 0, 180));
}

void Motor::setPWMVelocity(int velocity){
    // make sure the velocity is in the range of -255 to 255
    if(velocity > 255){
        velocity = 255;
    } else if(velocity < -255){
        velocity = -255;
    }

    currentVelocity = (float)velocity;

    if(velocity > 0){
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backwardPin, LOW);
    } else if(velocity < 0){
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, HIGH);
    } else {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, LOW);
    }
    ledcWrite(pwmChannel, abs(velocity));
}
