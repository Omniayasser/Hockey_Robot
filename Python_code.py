import serial
import time
import cv2
import numpy as np
import socket

#Define object specific variables
dist = 0
focal = 390
pixels = 30
width = 2.6
kernel = np.ones((3, 3), 'uint8')
font = cv2.FONT_HERSHEY_SIMPLEX
org = (0, 20)
fontScale = 0.6
color = (0, 0, 255)
thickness = 2

# find the distance from then camera
def get_dist(rectange_params, image):
    # find no of pixels covered
    pixels = rectange_params[1][0]
    #print(pixels)
    # calculate distance
    dist = ((width * focal) / pixels)
  #  time.sleep(2)
   # print(dist)

    # Wrtie n the image
    image = cv2.putText(image, 'Distance from Camera in CM :', org, font,
                        1, color, 2, cv2.LINE_AA)

    image = cv2.putText(image, str(dist), (110, 50), font,
                        fontScale, color, 1, cv2.LINE_AA)

    return image

class ObjectDetection:

    def __init__(self, arduino_port: str, camera_index: int):

        self.arduino_port = arduino_port
        self.camera_index = camera_index
        self.obstacle_detected = False

    def detect_obstacle(self):

        arduino = serial.Serial('COM6', 9600)


        # Initialize the camera

        cap = cv2.VideoCapture(1)
        global sora

        while True:
            ret, frame = cap.read()
            into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            L_limit_blue = np.array([98, 50, 50])  # setting the blue lower limit
            U_limit_blue = np.array([139, 255, 255])  # setting the blue upper limit
            b_mask = cv2.inRange(into_hsv, L_limit_blue, U_limit_blue)
            blue = cv2.bitwise_and(frame, frame, mask=b_mask)
            d_img = cv2.morphologyEx(b_mask, cv2.MORPH_OPEN, kernel, iterations=5)
            contours, hierarchy = cv2.findContours(d_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cont = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

            for cnt in cont:
                # check for contour area
                if (cv2.contourArea(cnt) > 100 and cv2.contourArea(cnt) < 306000):
                    # Draw a rectange on the contour
                    sora = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(sora)
                    box = np.int0(box)
                    cv2.drawContours(blue, [box], -1, (255, 0, 0), 3)
                    #print(rect[0][0])
                blue = get_dist(sora, blue)

            data = len(contours)
                if(sora[0][0]>=253 and sora[0][0]<=400):
                    data=0
                    self.obstacle_detected = True
                    arduino.write(b'0')
                elif(sora[0][0]<253 and sora[0][0]>0):
                    data = 1 #shayef object  yemen
                    arduino.write(b'1')
                elif (sora[0][0] > 400 and sora[0][0]>0):#shayef object shemal
                    arduino.write(b'2')
                elif(sora[0][0]<0):
                    arduino.write(b'3')


            cv2.imshow('Original', frame)  # to display the original frame
            cv2.imshow('Blue Detector', blue)  # to display the blue object output

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

                # Release the camera and close all windows
        cap.release()
        cv2.destroyAllWindows()

        return self.obstacle_detected

obj_detection = ObjectDetection('/dev/ttyACM0', 0)

        # Call the detect_obstacle function to start object detection
obstacle_detected = obj_detection.detect_obstacle()

        # Check if an obstacle is detected



















