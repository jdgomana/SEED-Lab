This program detects an Aruco marker in the camera feed and identifies which quadrant of the frame it is located in.  
The quadrant corresponds to a string value that represents wheel positions, which will be sent to the LCD screen and Arduino over I2C on position updates.  
The LCD logic utilizes threading to display the goal position without lag.  

Setup:  
See Mini Project README. Program can be run without Arduino wire connections.
