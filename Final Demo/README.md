Objective:  

This final demo showcases the full implementation of our robot’s vision-based navigation system using ArUco markers and directional arrows. 
The robot autonomously follows a path of markers by detecting a marker, rotating in the correct direction based on the arrow, moving to the next marker, and repeating until the sequence is complete.  


Setup:

1. Wiring connections:  
   LCD Screen <-> Pi  
   USB Camera <-> Pi  
   Pi Pin 2 <-> Arduino Pin A4  
   Pi Pin 3 <-> Arduino Pin A5  
   Pi Pin 6 <-> Arduino GND
   
   <p>
      <img width="250" height="150" alt="image" src="https://github.com/user-attachments/assets/ddda6e54-318a-4374-bb52-cce38aea2884" />  
   </p>
   <p>
      <img width="175" height="175" alt="image" src="https://github.com/user-attachments/assets/141fa792-8f9b-47ec-8e89-9b35b89c494f" />
   </p>

3. On the Pi, run final_demo.py. Ensure camera is connected and code is running correctly before continuing. Make sure power is not disconnected after starting the program.
4. Flash the Arduino with the final demo code. Turn robot off as it will immediately try to turn.
5. Once robot is positioned, turn it on and press reset button on Arduino to start the maze navigation. The "searching" phase always begins with the robot turning counter-clockwise.

