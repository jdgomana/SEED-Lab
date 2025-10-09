'''
This program detects an Aruco marker in the camera feed and identifies which quadrant of the frame it is located in.
The quadrant corresponds to a string value that represents wheel positions, which will be sent to the LCD screen and Arduino over I2C on 
position updates.
The LCD logic utilizes threading to display the goal position without lag.
'''

from smbus2 import SMBus
from time import sleep
import cv2
import numpy as np
from cv2 import aruco
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Goal position variables
NE = "00"
NW = "01"
SW = "11"
SE = "10"

# I2C position offset
POS_OFFSET = 0

# Initial positions
posCommand = NE
lastPosCommand = NE
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
        
# Write an I2C block
def writeI2C(offset, command):
        print("Sending: ", command)
        try:
                command = [ord(character) for character in command]
                i2c.write_i2c_block_data(ARD_ADDR, offset, command)
        # Continue program if wiring is incorrect
        except IOError:
                print("Arduino cannot be written to.")
        pass



# Initialize camera
camera = cv2.VideoCapture(0)
sleep(.5)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)

# Start LCD thread
lcdThread = threading.Thread(target=lcd,args=())
lcdThread.start()

# Set initial position as NE
writeI2C(POS_OFFSET, posCommand)
q.put(posCommand)

while (True):
        
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
        cv2.line(image, (int(FRAME_WIDTH/2), 0), (int(FRAME_WIDTH/2), int(FRAME_HEIGHT)), (0, 255, 0), 2)
        cv2.line(image, (0, int(FRAME_HEIGHT/2)), (int(FRAME_WIDTH), int(FRAME_HEIGHT/2)), (0, 255, 0), 2)
        cv2.imshow('Image', image)
        cv2.waitKey(1)

        # Set I2C command according to the quadrant marker is detected
        if markerXPos == None:
                pass
        elif markerXPos <= FRAME_WIDTH / 2: 
            if markerYPos <= FRAME_HEIGHT / 2:
                posCommand = NW
            else:
                posCommand = SW
        else:
            if markerYPos <= FRAME_HEIGHT / 2:
                posCommand = NE
            else:
                posCommand = SE

        # Only send I2C data if position changes
        if posCommand != lastPosCommand:
                writeI2C(POS_OFFSET, posCommand)
                q.put(posCommand)

        # Update position
        lastPosCommand = posCommand

