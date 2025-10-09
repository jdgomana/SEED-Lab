
// Assignment Demo: Spin DC Motors with PWM on pins 9/10
// Left motor uses pins 7 + 9
// Right motor uses pins 8 + 10
// Enable pin: 4

const int EN        = 4;   // Enable pin for motor driver shield

// Left motor
const int leftMdir  = 7;   // Direction pin
const int leftMpwm  = 9;   // PWM pin

// Right motor
const int rightMdir = 8;   // Direction pin
const int rightMpwm = 10;  // PWM pin

void setup() {
  // Set pins as outputs
  pinMode(EN, OUTPUT);

  pinMode(leftMdir, OUTPUT);
  pinMode(leftMpwm, OUTPUT);

  pinMode(rightMdir, OUTPUT);
  pinMode(rightMpwm, OUTPUT);

  // Enable motor driver
  digitalWrite(EN, HIGH);

  // Start with motors stopped
  digitalWrite(leftMdir, LOW);
  analogWrite(leftMpwm, 0);

  digitalWrite(rightMdir, LOW);
  analogWrite(rightMpwm, 0);

  Serial.begin(115200);
}

void loop() {
  // Spin LEFT motor forward at 50% duty  //for left high = back low= foward //right shoudl be reversed.
  digitalWrite(leftMdir, HIGH); 
  analogWrite(leftMpwm, 128);   // ~50% duty
  delay(3000);

  // Spin LEFT motor forward at 100%
  analogWrite(leftMpwm, 255);
  delay(3000);

  // Spin LEFT motor reverse at 50%
  digitalWrite(leftMdir, LOW);
  analogWrite(leftMpwm, 128);
  delay(3000);

  // Stop LEFT motor
  analogWrite(leftMpwm, 0);
  delay(2000);

  // --- Repeat for RIGHT motor ---
  digitalWrite(rightMdir, HIGH);
  analogWrite(rightMpwm, 128);   // 50%
  delay(3000);

  analogWrite(rightMpwm, 255);   // 100%
  delay(3000);

  digitalWrite(rightMdir, LOW);  // reverse
  analogWrite(rightMpwm, 128);
  delay(3000);

  analogWrite(rightMpwm, 0);     // stop
  delay(2000);
}