from Vision.Vision import Vision
import cv2

def testDevice(num_device_to_test):
    good_devices = []
    for i in range(num_device_to_test):
        cap = cv2.VideoCapture(i) 
        if cap is None or not cap.isOpened():
            pass
        else:
            good_devices.append(i)
    return good_devices

dev = testDevice(10)
print(dev)


cam = Vision(dev[1])

while True:
    if cam.drawTarget() == False:
        break
    
cam.cleanup()