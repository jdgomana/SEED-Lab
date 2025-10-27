// Demo 2b: Manual odometry only (motors OFF)
// Prints: time_s \t x_m \t y_m \t phi_rad

// ---- Motor driver pins (kept OFF) ----
const int EN         = 4;   // keep LOW so motors cannot run
const int leftMdir   = 7;   // not used
const int leftMpwm   = 9;   // stay 0
const int rightMdir  = 8;   // not used
const int rightMpwm  = 10;  // stay 0

// ---- Encoder pins (UNO) ----
const int leftEncA   = 2;   // interrupt 0
const int leftEncB   = 5;   // GPIO
const int rightEncA  = 3;   // interrupt 1
const int rightEncB  = 6;   // GPIO

// ---- Geometry & scaling ----
const long  COUNTS_PER_REV = 3200;            // 2π rad = 3200 counts
const float PI8            = 3.14159265f;
const float TWO_PI_F       = 2.0f * PI8;

const unsigned long Ts_ms  = 10;              // 100 Hz
const float Ts_s           = Ts_ms / 1000.0f;

const float WHEEL_RADIUS_M = 0.0762f;         // 3 in → meters
const float WHEEL_BASE_M   = 0.36195f;        // 14.25 in → meters

// ---- Per-wheel direction signs ----
// If forward motion still goes the wrong way for a wheel, flip its sign.
const int LEFT_SIGN  = +1;   // try -1 first (your left seemed reversed)
const int RIGHT_SIGN = +1;   // right looked normal

// ---- State ----
volatile long leftCount  = 0;
volatile long rightCount = 0;

float x_m = 0.0f, y_m = 0.0f, phi = 0.0f;    // phi wrapped to [0, 2π)

// ---- Helpers ----
static inline float wrap_0_2pi(float a){
  while (a < 0.0f)      a += TWO_PI_F;
  while (a >= TWO_PI_F) a -= TWO_PI_F;
  return a;
}

// ---- Encoder ISRs (standard rule; no baked-in inversion) ----
void ISR_leftA()  {
  int A = digitalRead(leftEncA);
  int B = digitalRead(leftEncB);
  leftCount  += (A == B) ? +1 : -1;
}
void ISR_rightA() {
  int A = digitalRead(rightEncA);
  int B = digitalRead(rightEncB);
  rightCount += (A == B) ? +1 : -1;
}

void setup() {
  // Ensure motors are OFF
  pinMode(EN, OUTPUT);          digitalWrite(EN, LOW);
  pinMode(leftMdir, OUTPUT);    pinMode(leftMpwm, OUTPUT);
  pinMode(rightMdir, OUTPUT);   pinMode(rightMpwm, OUTPUT);
  analogWrite(leftMpwm, 0);     analogWrite(rightMpwm, 0);

  // Encoders
  pinMode(leftEncA, INPUT_PULLUP);
  pinMode(leftEncB, INPUT_PULLUP);
  pinMode(rightEncA, INPUT_PULLUP);
  pinMode(rightEncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncA),  ISR_leftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncA), ISR_rightA, CHANGE);

  Serial.begin(115200);
  Serial.println("# time_s\tx_m\ty_m\tphi_rad");
}

void loop() {
  static unsigned long t0 = millis(), tLast = t0;
  unsigned long now = millis();
  if (now - tLast < Ts_ms) return;
  tLast = now;
  float time_s = (now - t0) / 1000.0f;

  // --- atomic snapshot of counts ---
  long cL, cR;
  noInterrupts();
  cL = leftCount;
  cR = rightCount;
  interrupts();

  // --- deltas since last sample ---
  static long pL = 0, pR = 0;
  long dCL = cL - pL;
  long dCR = cR - pR;
  pL = cL;  pR = cR;

  // --- counts -> wheel angle increments (rad), apply per-wheel sign here ---
  const float radPerCount = TWO_PI_F / (float)COUNTS_PER_REV;
  float dThL = (LEFT_SIGN  * dCL) * radPerCount;
  float dThR = (RIGHT_SIGN * dCR) * radPerCount;

  // --- wheel arc lengths (m) ---
  float dSL = WHEEL_RADIUS_M * dThL;
  float dSR = WHEEL_RADIUS_M * dThR;

  // --- differential-drive kinematics ---
  float dS   = 0.5f * (dSL + dSR);           // center distance moved
  float dPhi = (dSR - dSL) / WHEEL_BASE_M;   // heading change

  // midpoint integration for better small-angle accuracy
  float phi_mid = wrap_0_2pi(phi + 0.5f * dPhi);
  x_m  += dS * cosf(phi_mid);
  y_m  += dS * sinf(phi_mid);
  phi   = wrap_0_2pi(phi + dPhi);

  // --- log in requested format ---
  Serial.print(time_s, 3);  Serial.print('\t');
  Serial.print(x_m, 6);     Serial.print('\t');
  Serial.print(y_m, 6);     Serial.print('\t');
  Serial.println(phi, 6);
}