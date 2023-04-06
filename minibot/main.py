# from SerialInterface.SerialInterface import SerialInterface
import cv2

# open a webcam
cam = cv2.VideoCapture(0)

# filter out yellow in the hsv color space
low_yellow = (20, 100, 100)
high_yellow = (30, 255, 255)

# get the dimensions of the webcam
x_dim = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
y_dim = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

output = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (x_dim, y_dim))

while True:
    # read a frame from the webcam
    ret, frame = cam.read()

    # convert the frame to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # create a mask for the yellow color
    mask = cv2.inRange(hsv, low_yellow, high_yellow)
    
    # dilate the mask
    # make a circular kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.erode(mask, kernel, iterations=2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    mask = cv2.dilate(mask, kernel, iterations=2)

    # find the contours of the yellow object
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # filter out any contours that are less than 0.5% of the image
    temp = [contour for contour in contours if cv2.contourArea(contour) > 0.005 * x_dim * y_dim]
    contours = temp
    
    
    if len(contours) > 0:
        # find the center of the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        try:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            cx, cy = -10, -10
        
        # draw a circle at the center of the largest contour
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        
        # draw the remaining contours
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    # show the frame
    cv2.imshow('frame', frame)
    
    output.write(frame)

    # wait for a key press
    key = cv2.waitKey(1)

    # if the key is q, quit
    if key == ord('q'):
        break

cam.release()
output.release()
cv2.destroyAllWindows()