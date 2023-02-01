import serial
from time import sleep
from time import time

class RobotSerial:
    def __init__(self, comPort : str, baudRate : int, timeout : int = 1):
        self.ser = serial.Serial(comPort, baudRate, timeout)
        self.ser.open()
        sleep(2) #TODO: Remove this eventually
        return
    
    def getMotor(self):
        self.ser.write(b"!0;")
        return self.readMessage("MTR")
    
    def setMotor(self):
        return
    
    def getSonar(self):
        return
    
    def setSonar(self):
        return
    
    def getIR(self):
        return
    
    def readMessage(self, header : str = "") -> str:
        ''' Read a message from the serial port. If a header is specified, 
        it will look for that header before accepting a message.
        
        @param header: The header to look for. If not specified, it will accept any message.
        @return: The header as a string and a list of tokens as floats.
        '''
        msg = self.ser.readline()
        timer = time()
        while msg[0:len(header) + 1] != '!' + header:
            msg = self.ser.readline()
            if time() - timer > 1:
                raise TimeoutError("Timeout while reading message")
        header, tokens = self.__processMessage(msg)
        return header, tokens
    
    def __processMessage(self, msg : str):
        '''
        Process a message from the serial port. This will split the message into a header and a list of tokens.
        
        '''
        tokens = []
        # split up the message by the commas and stop when you reach the semicolon
        # the first character will be an exclamation point, so we start at 1
        token = ""
        for i in range(1, len(msg)):
            char = msg[i]
            if char == ',': # if we reach a comma, we have a token
                tokens.append(token)
                token = ""
            elif char == ';':
                break
            else:
                token += msg[i]
        
        header = tokens[0]
        tokens = tokens[1:]
        # convert all tokens to floats
        return header, [float(token) for token in tokens]
            
        