The Arduino sketch provided implements a per-wheel PI controller. 
Input: This Controller takes information in through an I2C reader, to trigger a single 180deg 
rotation that reacts and adjust to disturbances and holds a final corrected postion. 

Features

Cascaded control: position PI → target speed → velocity PI → signed motor voltage/PWM

Per-wheel tuning: independent Kp_pos/Ki_pos and Kp arrays

Hold/hysteresis logic: snaps to hold when within tight error/velocity bands

Encoder low-pass filtering: first-order filter on speed estimates

I²C input: single-byte triggers to add +π/2 to left/right targets (MY_ADDR = 8)

Serial telemetry: tab-separated avg|V|, left_theta, right_theta at 100 Hz

_______________________________________________________________________________________
Hardware

MCU: Arduino UNO (ATmega328P)

Driver: H-bridge with separate DIR/PWM per motor

Encoders: 2× quadrature (A/B)

Power: 7.5 V nominal (VBATT) to driver; logic 5 V to UNO

________________________________________________________


Pinout (UNO)
Function	Left	Right	            Notes
DIR        D7	   D8	            MDIR[2]
PWM        D9	   D10	          MPWM[2] (Timer-based PWM)
ENC A	     D2(INT0)	D3 (INT1)	  rising/falling interrupt
ENC B	     D5	 D6	              polled in ISR
ENABLE	   D4                 	—	HIGH = driver on
I²C	A4(SDA), A5(SCL)	—       	slave addr 8

______________________________________________
DEMO: 
Demo: 
Once Hardware is connected, when an aruco marker is detected or updated in one of four quadrants,
Arduino will receive two characters(a combination of 0 and 1) based on the read numbers
The Arduino will move, the left wheel, right wheel, both, or none based on the given table:

Marker visible in image quadrant   | Left Wheel | Right Wheel
-------------------------------------------------------------
NE                                 |     0      |     0
NW                                 |     0      |     1
SW                                 |     1      |     1
SE                                 |     1      |     0

