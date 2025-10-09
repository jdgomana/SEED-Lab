/*
This program demonstrates I2C communication between the Pi and Arduino.
*/

#include <Wire.h>
#define MY_ADDR 8

// Global variables to be used for I2C communication
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t offset = 0;
volatile uint8_t wheelOne = 0;
volatile uint8_t wheelTwo = 0;
volatile uint8_t newWheelOne = 0;
volatile uint8_t newWheelTwo = 0;

void setup() {
  Serial.begin(115200);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
}

void loop() {
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    printReceived();
    msgLength = 0;
  }
}

// printReceived helps us see what data we are getting from the leader
void printReceived() {
  newWheelOne = (char) instruction[0] - '0';
  newWheelTwo = (char) instruction[1] - '0';
  
  // Print on serial console
  Serial.print("Goal position: "); 
  Serial.print(String(newWheelOne) + String(newWheelTwo) + "\t");
  Serial.println("");

  if ( newWheelOne != wheelOne ) {
    // do something
    Serial.print("Moving wheel one");
    Serial.println("");
  }


  if ( newWheelTwo != wheelTwo ) {
    // do something
    Serial.print("Moving wheel two");
    Serial.println("");
  }

  wheelOne = newWheelOne;
  wheelTwo = newWheelTwo;
  
}

// function called when an I2C interrupt event happens
void receive() {
  offset = Wire.read();
  
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}
