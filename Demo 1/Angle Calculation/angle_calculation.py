# This program identifies an Aruco marker and the angle between its center and the camera center. Before running, make sure to include the
# file path to the provided npz file which contains the camera matrices.

from smbus2 import SMBus
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8


# Load camera matrices with file path
data = np.load(r'/home/seedlab/seed/Demo 1/camera_calibration_data.npz')
mtx = data['camera_matrix']
dist = data['dist_coeff']

# Initial marker positions
markerXPos = None
markerYPos = None
        
# Initialize I2C, Aruco dictionary, threading queue
i2c = SMBus(1)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
q = queue.Queue()

# Threading function that initializes LCD and waits on LCD updates
def lcd():
        
        lcd_columns = 16
        lcd_rows = 2
        board_i2c = board.I2C()
        lcd = character_lcd.Character_LCD_RGB_I2C(board_i2c, lcd_columns, lcd_rows)
        lcd.clear()
        lcd.color = [100, 0 ,0]

        while True:
                if not q.empty():
                        newMessage = q.get()
                        lcd.message = newMessage


# Initialize camera
camera = cv2.VideoCapture(0)
sleep(.5)

# Start LCD thread
lcdThread = threading.Thread(target=lcd,args=())
lcdThread.start()
        
while True:
        
        ret, image = camera.read()
        
        if not ret:
                print("Could not capture image from camera!")
                quit()

        # Aruco marker detection
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        corners,ids,rejected = aruco.detectMarkers(gray,aruco_dict)
        image = aruco.drawDetectedMarkers(image,corners,borderColor = 4)
        if not ids is None:
                ids = ids.flatten()
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        markerXPos = int(np.mean(markerCorners[:, 0]))
                        markerYPos = int(np.mean(markerCorners[:, 1]))
                        cv2.line(image, (markerXPos, markerYPos-5), (markerXPos, markerYPos+5), (0, 0, 255), 1)
                        cv2.line(image, (markerXPos-5, markerYPos), (markerXPos+5, markerYPos), (0, 0, 255), 1)
        cv2.imshow('Image', image)

        # Horizontal FOV calculations ( 2 * arctan( width / 2 * focal length ) )
        h, w = image.shape[:2]
        fx = mtx[0, 0]
        fovX = 2 * np.degrees(np.arctan(w / (2 * fx)))

        # Marker angle calculations ( (0.5 * FOV) * (The number of pixels from the center of the image to the pixel 
        # in the middle of the object, divided by the number of pixels from the center of the image to the edge of the image) )
        if markerXPos != None:
                centerToEdge = w / 2
                centerToCOM = centerToEdge - markerXPos
                angle = round((0.5 * fovX) * (centerToCOM / centerToEdge), 2)
                # Display on LCD screen 
                q.put("X Angle: " + str(angle))
                cv2.waitKey(500)
        
