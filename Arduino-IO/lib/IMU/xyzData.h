#pragma once
#include <Arduino.h>

struct xyzData{
    float x;
    float y;
    float z;

    xyzData(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }

    xyzData(){
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }

    xyzData operator+(xyzData other){
        return xyzData(this->x + other.x, this->y + other.y, this->z + other.z);
    }

    xyzData operator-(xyzData other){
        return xyzData(this->x - other.x, this->y - other.y, this->z - other.z);
    }

    xyzData operator*(float scalar){
        return xyzData(this->x * scalar, this->y * scalar, this->z * scalar);
    }

    xyzData operator/(float scalar){
        return xyzData(this->x / scalar, this->y / scalar, this->z / scalar);
    }

    float magnitude(){
        return sqrt(x*x + y*y + z*z);
    }

    void operator=(xyzData other){
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }

    float get(uint8_t index){
        switch(index){
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                return 0;
        }
    }

    void print(){
        Serial.print(x, 5);
        Serial.print(",");
        Serial.print(y, 5);
        Serial.print(",");
        Serial.print(z, 5);
    }
};