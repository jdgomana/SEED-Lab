'''
This program identifies a green shape from a picture and displays a new image with the green shape contoured.
'''

from time import sleep
import numpy as np
import cv2

# Initialize camera
camera = cv2.VideoCapture(0)
sleep(1)

ret, colors = camera.read()

if not ret:
	print("Could not capture image from camera!")
	quit()
else:
        # Show original image
        cv2.imshow('Image',colors)

        # Convert to HSV
        colorsHSV = cv2.cvtColor(colors,cv2.COLOR_BGR2HSV)

        # Apply mask with green values
        upperGreen = np.array([75,255,255])
        lowerGreen = np.array([60,100,30])
        mask = cv2.inRange(colorsHSV,lowerGreen,upperGreen)

        # Close gaps then dilate
        kernel = np.ones((5,5),np.uint8)
        greenClosed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        greenClosed = cv2.dilate(greenClosed, kernel, iterations=1)

        # Create contour of shape
        contourGreenVisualize = colors.copy()
        contoursGreen,_ = cvh2.findContours(greenClosed,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(contourGreenVisualize,contoursGreen,-1,(255,0,0),3)
        cv2.imshow("Contour",contourGreenVisualize)

        # Destroy windows on button press
        cv2.waitKey(0)
        cv2.destroyAllWindows()

