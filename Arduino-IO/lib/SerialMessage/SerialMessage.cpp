#include "SerialMessage.h"

void SerialMessage::readSerial(){
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char c;

    // read the incoming serial data:
    while (Serial.available() > 0 && data_recieved == false) {
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
                data_recieved = true;
            }
        }
        // if the incoming character is the startMarker, set the recvInProgress flag
        else if (c == startMarker) {
            recvInProgress = true;
        }
    }
}

void SerialMessage::parseData() {      // split the data into its parts
    this->populated_args = 0; // reset the populated args counter
    char * indx; // this is used by strtok() as an index
    int i = 0;
    indx = strtok(temp_data, ",");      // get the first part - the string
    while(indx != NULL){
        this->args[i] = atoi(indx);
        populated_args++;
        i++;
        indx = strtok(NULL, ","); // this continues where the previous call left off
    }
}

void SerialMessage::update(){
    readSerial();
    if (data_recieved == true) {
        strcpy(temp_data, data);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        //printArgs();

        // TODO: add code to do something with the data
        data_recieved = false;
        new_data = true;
    }
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

int SerialMessage::getPopulatedArgs(){
    return populated_args;
}

void SerialMessage::printArgs(){
    Serial.print("Current number of args: ");
    Serial.println(populated_args);
    for (int i = 0; i < populated_args; i++) {
        Serial.print(args[i]);
        Serial.print(" ");
    }
    Serial.println();
}