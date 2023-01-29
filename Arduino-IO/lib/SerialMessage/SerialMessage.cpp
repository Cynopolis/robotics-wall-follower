#include "SerialMessage.h"
#include "Arduino.h"

SerialMessage::SerialMessage(int baud_rate){
    Serial.begin(baud_rate);
    // initialize args array to all 0's
    for (int i = 0; i < args_length; i++) {
        args[i] = 0;
    }
}

void SerialMessage::readSerial(){
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char c;

    // read the incoming serial data:
    while (Serial.available() > 0 && new_data == false) {
        // get the neext character in the serial buffer
        c = Serial.read();

        // only execute this if the startMarker has been received
        if (recvInProgress == true) {
            // if the incoming character is not the endMarker...
            if (c != endMarker) {
                // add it to the data array
                data[ndx] = c;
                ndx++; // increment the data array index
                // if the index is greater than the maximum data array size,
                // keep overwriting the last element until the endMarker is received.
                if (ndx >= num_chars) {
                    ndx = num_chars - 1;
                }
            }
            // if the incoming character is the endMarker clean up and set the flags
            else {
                data[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                new_data = true;
            }
        }
        // if the incoming character is the startMarker, set the recvInProgress flag
        else if (c == startMarker) {
            recvInProgress = true;
        }
    }
}

void SerialMessage::parseData() {      // split the data into its parts

    char * indx; // this is used by strtok() as an index
    int i = 0;
    indx = strtok(temp_data, ",");      // get the first part - the string

    while(indx != NULL){
        this->args[i] = atoi(indx);
        i++;
        indx = strtok(NULL, ","); // this continues where the previous call left off
    }
}

void SerialMessage::update(){
    readSerial();
    if (new_data == true) {
        strcpy(temp_data, data);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parseData();

        // TODO: add code to do something with the data
        new_data = false;
    }
}

void SerialMessage::println(char message[]){
    Serial.println(message);
}

void SerialMessage::print(char message[]){
    Serial.print(message);
}

bool SerialMessage::isNewData(){
    return new_data;
}

void SerialMessage::clearNewData(){
    new_data = false;
}

int * SerialMessage::getArgs(){
    return args;
}

int SerialMessage::getArgsLength(){
    return args_length;
}