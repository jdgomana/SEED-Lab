Objective:  
The objective of Demo 1 is to accomplish three main tasks:
1. Detect the Aruco marker and report its horizontal angle between the camera center and marker center on the LCD screen.
2. Move forward in a straight line and stop after a specified distance in feet.
3. Rotate the robot by a specified angle in degrees, and move forward a specified distance in feet.

Setup:  
No connections are needed between the Pi and Arduino.

<img width="365" height="290" alt="image" src="https://github.com/user-attachments/assets/bfad7ef6-c81a-4c17-bbaa-1c792fc2cd98" />



Controls: A PID controller needs to be implemented in order to control the body of the robot as a whole system instead of a wheel-by-wheel basis. 
A 
Functional State machine that takes the robot's initial position, then rotates the robot to the desired heading, then moves forward by a specified distance, and stops and readies itself for the  next command. 

INSIDE Demo1 -> there is a folder which houses the code ( Demo1AnnotatedCode...) and a .txt that describes the new and additional functionalities from the minilab project (Added Functionalaties.txt)
