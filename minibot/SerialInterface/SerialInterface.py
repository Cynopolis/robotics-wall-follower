import serial
import time

class SerialInterface:
    def __init__(self, com, baud_rate) -> None:
        '''
        Initializes the IMU class.
        args:
        - ser: An initalized serial object which can communicate with the IMU
        '''
        self.serial = serial.Serial(com, baud_rate)
        return
    

    def process_message(self, message : str):
        '''
        Sorts and parses data.
        args:
        - message: A string that stores header and parsed data.
        '''
        if(len(message) == 0): 
            return "", []
        if(message[0] != '!'):
            return "", []
        if(message[len(message)-1] != ';'):
            return "", []
        
        parsed_data = []
        token = ""
        for letter in message[1:]:
            if letter == ';':
                parsed_data.append(token)
                break
            if letter == ',':
                parsed_data.append(token)
                token = ""
                continue
            token = token+letter
        header = parsed_data[0]
        parsed_data = parsed_data[1:]
        for i,num in enumerate(parsed_data):
            try:
                parsed_data[i] = float(num)
            except:
                pass
        
        return header, parsed_data

    def getItem(self, command, return_key, timeout=1):
        return_message = ""
        # create the command and write it to the serial
        command = ("!"+command+";\n").encode('UTF-8')
        
        # wait for the return message to be recieved or the timeout to be reached
        timer = time.time()
        while return_message.find(return_key) == -1 and time.time()-timer < timeout:
            self.serial.write(command)
            return_message = self.serial.readline().decode('UTF-8').strip()
        
        # parse the return message
        header, data = self.process_message(return_message)
        
        # check if the header is the correct one
        if header != return_key:
            return None
        
        # return the data
        return data
    
    def setItem(self, command, return_key, timeout=1):
        # create the command and write it to the serial
        command = command.encode('UTF-8')
        timer = time.time()
        return_msg = ""
        # keep writing until the response is recieved or the timeout is reached
        while return_msg.find(return_key) == -1 and time.time()-timer < timeout:
            self.serial.write(command)
            return_msg = self.serial.readline().decode('UTF-8').strip()
        
        header, _ = self.process_message(return_msg)
        
        if header != return_key:
            return False
        
        return True
    
    def getPose(self):
        respone = None
        while respone is None:
            respone = self.getItem("1", "POSE_READ")
        return respone
        
    def setTargetPose(self, velocity, angle):
        message = "!2,"+str(velocity)+","+str(angle * 180 / 3.14159265359)+";\n"
        return self.setItem(message, "POSE_WRT")
    
    def setVelocityConstants(self, kp, ki, kd):
        message = "!3,"+str(kp)+","+str(ki)+","+str(kd)+";\n"
        return self.setItem(message, "VELWRT")
    
    def setAngleConstants(self, kp, ki, kd):
        message = "!4,"+str(kp)+","+str(ki)+","+str(kd)+";\n"
        return self.setItem(message, "ANGLEWRT")
    
    def setMotorConstants(self, motor_number, kp, ki, kd):
        message = "!5,"+str(motor_number)+str(kp)+","+str(ki)+","+str(kd)+";\n"
        
        if motor_number == 0:
            return self.setItem(message, "LEFTMTRPID")
        elif motor_number == 1:
            return self.setItem(message, "RIGHTMTRPID")
        
        return False
    


if __name__ == "__main__":
    ser = SerialInterface('/dev/ttyUSB0', 115200)
    # ser = SerialInterface('COM5', 115200)
    print("Starting")
    #time.sleep(5)
    
    response = None
    while response == None:
        response = ser.getPose()
        print(response)
    print("Setting target pose to 100 velocity and 90 degrees")
    print(ser.setTargetPose(100, 3.14159265359/2))
    time.sleep(5)
    print("Setting target pose to 100 velocity and 90 degrees")
    print(ser.setTargetPose(1000, 0))
    time.sleep(5)
    print("Stopping motors")
    print(ser.setTargetPose(0, 0))
    print("final pose:")
    print(ser.getPose())

        