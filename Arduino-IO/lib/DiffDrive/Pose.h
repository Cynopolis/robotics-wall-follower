#pragma once
#include <Arduino.h>

// typedef struct pose
typedef struct Pose{
    Pose() : x(0), y(0), theta(0), d_x(0), d_y(0), d_theta(0) {}
    Pose(float x, float y, float theta, float d_x, float d_y, float d_theta) : x(x), y(y), theta(theta), d_x(d_x), d_y(d_y), d_theta(d_theta) {}
    // x and y are in mm
    float x = 0;
    float y = 0;
    // theta is in radians
    float theta = 0;
    // d_x, d_y, and d_theta are the change in position and theta in mm/s and radians/s
    float d_x = 0;
    float d_y = 0;
    float d_theta = 0;

    // Add a function to add two poses together
    Pose operator+(const Pose &other){
        Pose newPose;
        newPose.x = this->x + other.x;
        newPose.y = this->y + other.y;
        newPose.theta = this->theta + other.theta;
        newPose.d_x = this->d_x + other.d_x;
        newPose.d_y = this->d_y + other.d_y;
        newPose.d_theta = this->d_theta + other.d_theta;
        return newPose;
    }

    // Add a function to subtract two poses together
    Pose operator-(const Pose &other){
        Pose newPose;
        newPose.x = this->x - other.x;
        newPose.y = this->y - other.y;
        newPose.theta = this->theta - other.theta;
        newPose.d_x = this->d_x - other.d_x;
        newPose.d_y = this->d_y - other.d_y;
        newPose.d_theta = this->d_theta - other.d_theta;
        return newPose;
    }

    // Add a function to make a pose equal to another pose
    Pose operator=(const Pose &other){
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
        this->d_x = other.d_x;
        this->d_y = other.d_y;
        this->d_theta = other.d_theta;
        return *this;
    }

    // add a function to multiply the pose by a constant
    Pose operator*(const float constant){
        Pose newPose;
        newPose.x = this->x * constant;
        newPose.y = this->y * constant;
        newPose.theta = this->theta * constant;
        newPose.d_x = this->d_x * constant;
        newPose.d_y = this->d_y * constant;
        newPose.d_theta = this->d_theta * constant;
        return newPose;
    }

    // add a function to divide the pose by a constant
    Pose operator/(const float constant){
        Pose newPose;
        newPose.x = this->x / constant;
        newPose.y = this->y / constant;
        newPose.theta = this->theta / constant;
        newPose.d_x = this->d_x / constant;
        newPose.d_y = this->d_y / constant;
        newPose.d_theta = this->d_theta / constant;
        return newPose;
    }

    const float get(uint8_t index){
        switch(index){
            case 0:
                return this->x;
            case 1:
                return this->y;
            case 2:
                return this->theta;
            case 3:
                return this->d_x;
            case 4:
                return this->d_y;
            case 5:
                return this->d_theta;
            default:
                return 0;
        }
    }

    void set(uint8_t index, float value){
        switch(index){
            case 0:
                this->x = value;
                break;
            case 1:
                this->y = value;
                break;
            case 2:
                this->theta = value;
                break;
            case 3:
                this->d_x = value;
                break;
            case 4:
                this->d_y = value;
                break;
            case 5:
                this->d_theta = value;
                break;
            default:
                break;
        }
    }

    const void print(){
        Serial.print("x: ");
        Serial.print(this->x,5);
        Serial.print(" y: ");
        Serial.print(this->y,5);
        Serial.print(" theta: ");
        Serial.print(this->theta,5);
        Serial.print(" d_x: ");
        Serial.print(this->d_x);
        Serial.print(" d_y: ");
        Serial.print(this->d_y);
        Serial.print(" d_theta: ");
        Serial.println(this->d_theta);
    }
} Pose;