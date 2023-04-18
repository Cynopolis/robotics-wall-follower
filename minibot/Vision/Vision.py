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

class Vision:
    
    def __init__(self, camera = 0):
        if camera == 0:
            cams = testDevice(10)
            self.cam = cv2.VideoCapture(cams[len(cams)-1])
        else:
            self.cam = cv2.VideoCapture(camera)
        # get the width and height of the camera
        self.width = int(self.cam.get(3))
        self.height = int(self.cam.get(4))
        self.target_aquired = False
        self.last_frame = None
        self.target = [0, 0, 0]
        self.no_target_found_count = 0
        return
    
    def __get_objects(self):
        ret, frame = self.cam.read()
        if not ret:
            return False
        self.last_frame = frame
        
        # convert the frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        #create a mask for the given color
        mask = cv2.inRange(hsv, self.low_color, self.high_color)
        
        # dilate the mask
        # make a circular kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        mask = cv2.erode(mask, kernel, iterations=2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        mask = cv2.dilate(mask, kernel, iterations=2)
        cv2.imshow("mask", mask)

        # find the contours of the yellow object
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # return a list of contours  that are greater than 0.5% of the image
        return [contour for contour in contours if cv2.contourArea(contour) > 0.005 * self.width * self.height]
        
        
    
    def aquire_target(self, low_color=(20, 100, 100), high_color=(30, 255, 255)):
        '''
        Aquires a target of the given color from the camera. The default color is yellow
        returns: true if a target was found, false otherwise
        '''
        self.low_color = low_color
        self.high_color = high_color
        
        contours = self.__get_objects()
        if contours == False:
            return False
        if len(contours) == 0:
            return False
        
        # find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        self.target_area = cv2.contourArea(largest_contour)/(self.width*self.height)
        self.target_aquired = True
        return True
    
    def getTarget(self):
        '''
        get the x,y coordinates of the target and the area of the target
        '''
        if not self.target_aquired:
            return False
        
        contours = self.__get_objects()
        if contours == False:
            self.no_target_found_count += 1
            if self.no_target_found_count > 10:
                self.target_aquired = False
                return False
            return self.target
        if len(contours) == 0:
            self.no_target_found_count += 1
            if self.no_target_found_count > 10:
                self.target_aquired = False
                return False
            return self.target
        
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)/(self.width*self.height)
        
        M = cv2.moments(largest_contour)
        try:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            self.no_target_found_count += 1
            if self.no_target_found_count > 10:
                self.target_aquired = False
                return False
            return self.target
        
        self.no_target_found_count = 0
        low_pass = 0.5
        self.target[0] = low_pass * self.target[0] + (1 - low_pass) * cx
        self.target[1] = low_pass * self.target[1] + (1 - low_pass) * cy
        self.target[2] = low_pass * self.target[2] + (1 - low_pass) * area
        return self.target
    
    def getImg(self):
        '''
        returns the last frame from the camera
        '''
        
        return self.last_frame
    
    def drawTarget(self, img=None):
        '''
        draws a circle on the image at the given target
        '''
        if img == None:
            try:
                if self.last_frame == None:
                    ret, img = self.cam.read()
            except ValueError:
                img = self.last_frame
                
        
        target = self.getTarget()
        if target != False:
            img = img.copy()
            cv2.circle(img, (int(target[0]), int(target[1])), 10, (0, 255, 0), -1)
        
        cv2.imshow('img', img)
        # if the user presses the escape key, exit
        if cv2.waitKey(1) == 27:
            return False
        
        return True

    def cleanup(self):
        '''
        cleans up the camera
        '''
        self.cam.release()
        cv2.destroyAllWindows()
        return
        
            
    
    
        
        
        