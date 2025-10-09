Objective:

The objective of this Mini Project is to begin integration between motor control and computer vision.  
An Aruco marker can be detected in the camera feed, and goal positions for the wheels are set according to which quadrant the marker is detected. This goal position is displayed on the LCD screen and transmitted to the Arduino.  
On position update, both wheels can move independently to the desired position. Integral control is also implemented to ensure the wheels are able to reject disturbances and return to the correct position.

Setup:
LCD Screen <-> Pi  
Pi Pin 2 <-> Arduino Pin A4  
Pi Pin 3 <-> Arduino Pin A5  
Pi Pin 6 <-> Arduino GND

<p>
  <img width="516" height="296" alt="image" src="https://github.com/user-attachments/assets/ddda6e54-318a-4374-bb52-cce38aea2884" />  
</p>  
  <img width="363" height="341" alt="image" src="https://github.com/user-attachments/assets/141fa792-8f9b-47ec-8e89-9b35b89c494f" />







