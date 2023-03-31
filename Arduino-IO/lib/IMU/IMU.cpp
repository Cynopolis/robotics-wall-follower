#include "IMU.h"

void IMU::begin(){
    Wire.begin();
    //Initialize the IMU
    uint8_t count = 0;
    while (!imu.begin()){
        Serial.println("Failed to connect to LSM9DS1. Attempt (" + String(count) + "/5)");
        count++;
        if(count > 5){
            return;
        }
    }
    this->isInitialized = true;
    Serial.println("LSM9DS1 online!");
    lastUpdate = millis();
    return;
}

void IMU::calibrate(){
    if(!isInitialized){
        Serial.println("IMU not initialized. Cannot calibrate.");
        return;
    }
    Serial.println("Calibrating IMU");
    xyzData accelSum = xyzData();
    xyzData gyroSum = xyzData();
    xyzData magSum = xyzData();
    for(int i = 0; i < 1000; i++){
        update();
        accelSum = accelSum + accel;
        gyroSum = gyroSum + gyro;
        magSum = magSum + mag;
    }
    this->accelOffset = accelSum / 1000;
    this->gyroOffset = gyroSum / 1000;
    this->magOffset = magSum / 1000;
    this->isCalibrated = true;
    Serial.println("Calibration complete");
}

void IMU::update(){
    if(!isInitialized){
        Serial.println("IMU not initialized. Cannot update.");
        return;
    }
    unsigned long now = millis();
    if(now - lastUpdate < 1){
        return;
    }
    float dt = float(now - lastUpdate) / 1000.0;

    imu.readAccel();
    imu.readGyro();
    imu.readMag();

    this->accel = xyzData(imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
    this->gyro = xyzData(imu.calcGyro(imu.gx)*deg_to_rad, imu.calcGyro(imu.gy)*deg_to_rad, imu.calcGyro(imu.gz)*deg_to_rad);
    this->mag = xyzData(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
    
    if(isCalibrated){
        this->accel = this->accel - this->accelOffset;
        this->gyro = this->gyro - this->gyroOffset;
        this->mag = this->mag - this->magOffset;
    }

    // Calculate the new orientation
    xyzData newOrientation = orientation + gyro * dt;
    // wrap the angles between -pi and pi
    newOrientation.x = wrap_angle(newOrientation.x);
    newOrientation.y = wrap_angle(newOrientation.y);
    newOrientation.z = wrap_angle(newOrientation.z);
    // Update the orientation
    this->orientation = newOrientation;
}

void IMU::print(){
    if(!isInitialized){
        Serial.println("IMU not initialized. Cannot print.");
        return;
    }
    Serial.print("!ACC,");
    accel.print();
    Serial.println(";");
    Serial.print("!GYR,");
    gyro.print();
    Serial.println(";");
    Serial.print("!MAG,");
    mag.print();
    Serial.println(";");
    Serial.print("!ORI,");
    orientation.print();
    Serial.println(";");
}

// bound between -pi and pi
float IMU::wrap_angle(float angle) {
    // First, wrap the angle between -2*pi and 2*pi radians
    angle = fmod(angle + PI, TAU) - PI;

    // Then, bound the angle between -pi and pi radians
    if (angle < -PI) {
        angle += TAU;
    } else if (angle > PI) {
        angle -= TAU;
    }

    return angle;
}