#include "IMU.h"

void IMU::begin(){
    Wire.begin();
    //Initialize the IMU
    uint8_t count = 0;
    while (!imu.begin(ACC_ADDRESS, MAG_ADDRESS, Wire)){
        Serial.println("Failed to connect to LSM9DS1. Attempt (" + String(count) + "/5)");
        count++;
        if(count > 5){
            return;
        }
    }
    this->isInitialized = true;
    calibrate();
    Serial.println("LSM9DS1 online!");
    lastUpdate = millis();
    return;
}

void IMU::calibrate(){
    if(!isInitialized){
        Serial.println("IMU not initialized. Cannot calibrate.");
        return;
    }

    Serial.println("Calibrating Gyro");
    // wait for a little while to allow the gyro to settle.
    delay(10);
    // add up 1000 gyro readings
    int i = 0;
    while(i < 1000){
        if ( imu.gyroAvailable() ){
            i++;
            imu.readGyro();
            gyroOffset.x += imu.calcGyro(imu.gx);
            gyroOffset.y += imu.calcGyro(imu.gy);
            gyroOffset.z += imu.calcGyro(imu.gz);
            
        }
    }
    // divide the sum by 1000 to get the average
    gyroOffset = gyroOffset / 1000.0;

    i = 0;
    while(i < 1000){
        if(imu.accelAvailable()){
            i++;
            imu.readAccel();
            accelOffset.x += imu.calcAccel(imu.ax);
            accelOffset.y += imu.calcAccel(imu.ay);
            accelOffset.z += imu.calcAccel(imu.az);
        }
    }
    accelOffset = accelOffset / 1000.0;

    Serial.print("Accel Offset: ");
    accelOffset.print();
    Serial.println();
    Serial.print("Gyro Offset: ");
    gyroOffset.print();
    Serial.println();
    Serial.print("Mag Offset: ");
    magOffset.print();
    Serial.println();
    Serial.println("Calibration complete");
}

void IMU::update(){
    if(!isInitialized){
        Serial.println("IMU not initialized. Cannot update.");
        return;
    }
    if(millis() - lastUpdate < 1){
        return;
    }

    if(imu.accelAvailable()){
        imu.readAccel();
        this->accel = xyzData(imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)) - accelOffset;
        if(isCalibrated) this->accel = this->accel - this->accelOffset;
    }

    if(imu.gyroAvailable()){
        imu.readGyro();
        update_angle();
    }

    if(imu.magAvailable()){
        imu.readMag();
        this->mag = xyzData(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        if(isCalibrated) this->mag = this->mag - this->magOffset;
    }
}

void IMU::update_angle(){
    // calculate the change in time since the last time this function was called
    double dt = double(millis() - lastGyroUpdate) / 1000;
    lastGyroUpdate = millis();

    if(isGyroFrozen){
        // xyzData temp = xyzData(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
        // this->gyroOffset = this->gyroOffset*0.999 + temp*0.001;
        return;
    }
    // calculate the gyro values and make sure to subtract the offset
    this->gyro.x = (double(imu.calcGyro(imu.gx)) - gyroOffset.x)*dt;
    this->gyro.y = (double(imu.calcGyro(imu.gy)) - gyroOffset.y)*dt;
    this->gyro.z = (double(imu.calcGyro(imu.gz)) - gyroOffset.z)*dt;

    // Update the orientation
    this->orientation = this->orientation + this->gyro * deg_to_rad;

    // wrap the angles between -pi and pi
    orientation.x = wrap_angle(orientation.x);
    orientation.y = wrap_angle(orientation.y);
    orientation.z = wrap_angle(orientation.z);
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

xyzData IMU::getOrientation(){
    return orientation;
}

xyzData IMU::getAccel(){
    return accel;
}

xyzData IMU::getGyro(){
    return gyro;
}

xyzData IMU::getMag(){
    return mag;
}