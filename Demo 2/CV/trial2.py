# This program uses distance and angle calculations to send movement commands to the Arduino. Arrow detection is also added.
# The FSM proceeds as follows: calculates angle to ArUco marker until close to 0 -> calculates distance to marker until 1.5ft -> calculates angle again until closer to zero -> detect direction of arrow -> end

from smbus2 import SMBus
from time import sleep
import numpy as np
import cv2
from cv2 import aruco

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8
ANGLE_TOLERANCE = 0.5
ALIGN_TOLERANCE = 0.25
DISTANCE_CALIBRATION = 111
TARGET_DISTANCE = 1.5
AREA_THRESH = 50

# Load camera matrices
data = np.load(r'/home/seedlab/seed/Demo 2/camera_calibration_data.npz')
mtx = data['camera_matrix']
dist = data['dist_coeff']
fx = mtx[0, 0]
fy = mtx[1, 1]

# Create a template for the arrow contour
template = cv2.imread(r'/home/seedlab/seed/Demo 2/arrow_template.png', cv2.IMREAD_GRAYSCALE)
_, templateBin = cv2.threshold(template, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
templateContours, _ = cv2.findContours(templateBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
arrowContour = max(templateContours, key=cv2.contourArea)

# Initial marker positions
markerXPos = None
markerYPos = None

# Initialize I2C and Aruco dictionary
i2c = SMBus(1)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# Initialize camera
camera = cv2.VideoCapture(0)
sleep(.5)



# Write an I2C block
def writeI2C(offset, command):
        print("Sending: ", command)
        try:
                command = [ord(character) for character in command]
                i2c.write_i2c_block_data(ARD_ADDR, offset, command)
        # Continue program even if wiring is incorrect
        except IOError:
                print("Arduino cannot be written to.")
        pass

# Calculate angle between the center of the camera and center of the Aruco marker
def calculateAngle():
    centerToEdge = w / 2
    centerToCOM = centerToEdge - markerXPos
    angle = round((0.5 * fovX) * (centerToCOM / centerToEdge), 1)
    print("Angle: " + str(angle))
    return angle



# CALCULATING_ANGLE state
# Robot rotates until the camera angle is close to zero (Aruco marker is perpendicular to the robot),
# with a tolerance of 0.5 degrees
def state0():
    if markerXPos != None:
        angle = calculateAngle()
        if abs(angle) <= ANGLE_TOLERANCE:
            writeI2C(0, "LOCALIZE_END")
            return state1
        else:
            return state0
    else:
        return state0

# CALCULATING_DISTANCE state
# Robot moves forward until the distance between the camera and marker is 1.5 ft
def state1():
    # Width calculations
    bottomLeft = markerCorners[2]
    bottomRight = markerCorners[3]
    pixelWidth = np.linalg.norm(bottomRight - bottomLeft)
    distance = round(1 * (DISTANCE_CALIBRATION / pixelWidth), 2)
    print("Feet: " + str(distance))
    if distance <= TARGET_DISTANCE:
        writeI2C(0, "FORWARD_END")
        return state2
    else:
        return state1

# ALIGNING state
# Robot rotates to fine-tune its angle to the marker. The tolerance is lowered to 0.25 degrees
def state2():
    angle = calculateAngle()
    if abs(angle) <= ALIGN_TOLERANCE:
        writeI2C(0, "ALIGN_END")
        return state3
    else:
        return state2
            
# DETECTING_ARROW state
# Arrow is detected, which Arduino will turn 90 degrees in the direction it points
def state3():
    # Arrow detection
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bestScore = float("inf")
    bestContour = None
    bestCenter = None
    
    for contour in contours:
        # Filter out tiny contours
        area = cv2.contourArea(contour)
        if area < AREA_THRESH:
            continue

        # Use matchShapes to determine shapes closest to arrow template
        score = cv2.matchShapes(arrowContour, contour, cv2.CONTOURS_MATCH_I1, 0)    
        if score < bestScore:
            bestScore = score
            bestContour = contour

    if bestContour is not None:
        M = cv2.moments(bestContour)
        if M["m00"] != 0:
            arrowXPos = int(M["m10"] / M["m00"])
            arrowYPos = int(M["m01"] / M["m00"])
            cv2.drawContours(image, bestContour, -1, (0, 255, 0), 1)
            cv2.circle(image, (arrowXPos, arrowYPos), 5, (255,0,0), -1)
            if arrowXPos < markerXPos:
                writeI2C(0, "L")
            else:
                writeI2C(0, "R")
        return None
    else:
        return state3

# Dictionary to describe the states
state_dictionary = {
    state0 : "CALCULATING_ANGLE",
    state1 : "CALCULATING_DISTANCE",
    state2 : "ALIGNING",
    state3 : "DETECTING_ARROW"
}

# Initial state
state = state0


        
while True:
        
        ret, image = camera.read()
    
        if not ret:
                print("Could not capture image from camera!")
                quit()

        # FOV calculations
        h, w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        image = cv2.undistort(image, mtx, dist, None, newcameramtx)
        fovX = 2 * np.degrees(np.arctan(w / (2 * fx)))
        fovY = 2 * np.degrees(np.arctan(h / (2 * fy)))
        
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
            
        # Updates states with each frame
        newState = state()
        state = newState
        cv2.imshow('Image', image)
        
        # Break while loop at end of FSM
        if state == None:
            break
        
        cv2.waitKey(1)
