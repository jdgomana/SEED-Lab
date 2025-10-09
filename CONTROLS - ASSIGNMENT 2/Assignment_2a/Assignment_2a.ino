// === VelocityControl_PI_2a.ino ===
//
// Purpose: 2a demo. Run a PI velocity controller from rest.
//          Log regular samples for t in [0, 3 s] to the serial port.
//          Can print either 6 columns (both wheels) or the 3 required columns.
//
// PRINT MODE: set to 6 for: time  V  omegaL  omegaR  thetaL  thetaR
//             set to 3 for: time  V  omegaAvg
#define PRINT_COLUMNS 6   // <-- change to 3 if your grader wants exactly 3 cols

// ---- Motor driver pins ----
const int EN         = 4;   // HIGH enables motor driver
const int leftMdir   = 7;
const int leftMpwm   = 9;
const int rightMdir  = 8;
const int rightMpwm  = 10;

// ---- Encoder pins (UNO) ----
const int leftEncA   = 2;   // interrupt 0
const int leftEncB   = 5;
const int rightEncA  = 3;   // interrupt 1
const int rightEncB  = 6;

// ---- Geometry & scaling ----
const long  COUNTS_PER_REV = 3200;        // 2π rad = 3200 counts
const float PI8            = 3.14159265f;
const float TWO_PI_F       = 2.0f * PI8;

// ---- Timing ----
const unsigned long desired_Ts_ms = 10;   // 100 Hz loop
const float Ts_s = (float)desired_Ts_ms / 1000.0f;

// ---- Per-wheel sign (flip if forward is negative) ----
const int LEFT_SIGN  = 1;
const int RIGHT_SIGN = +1;

// ---- Plant / electrical ----
const float BATTERY_VOLTAGE = 7.5f;      // << set to your pack
const float VMAX            = 0.95f * BATTERY_VOLTAGE; // headroom vs saturation

// ---- Desired speed (TA-provided), step at t=1s ----
const float OMEGA_REF_RAD_S = 12.0f;      // << put your assigned rad/s here

// ---- PI controller (tune these) ----
// For first-order motor G(s)=K/(s+sigma), PI often: Kp ≈ (sigma/K)*M, Ki ≈ (sigma^2/K)*N.
// Start conservative; adjust on the bench.
float Kp = 0.9f;      // proportional gain
float Ki = 8.0f;      // integral gain [1/s]

// ---- Simple velocity low-pass (to reduce encoder quantization noise) ----
// 1st-order IIR: y = a*y + (1-a)*x, with a = exp(-2π*fc*Ts)
const float fc_hz = 8.0f;            // cutoff (adjust 5–15 Hz as needed)
const float a_lp  = expf(-2.0f*PI8*fc_hz*Ts_s);

// ---- Encoder state (ISR-updated) ----
volatile long leftCount  = 0;
volatile long rightCount = 0;

// ---- Prior positions (radians) for velocity calc ----
float thetaL_prev = 0.0f, thetaR_prev = 0.0f;

// ---- Filtered velocity (state) ----
float omegaL_f = 0.0f, omegaR_f = 0.0f;

// ---- Integrator (PI) ----
float integ = 0.0f;

// ---- Encoder ISRs ----
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
  // Motors enabled, forward direction
  pinMode(EN, OUTPUT);          digitalWrite(EN, HIGH);
  pinMode(leftMdir, OUTPUT);    digitalWrite(leftMdir, HIGH);
  pinMode(rightMdir, OUTPUT);   digitalWrite(rightMdir, HIGH);
  pinMode(leftMpwm, OUTPUT);    analogWrite(leftMpwm, 0);
  pinMode(rightMpwm, OUTPUT);   analogWrite(rightMpwm, 0);

  // Encoders
  pinMode(leftEncA, INPUT_PULLUP);
  pinMode(leftEncB, INPUT_PULLUP);
  pinMode(rightEncA, INPUT_PULLUP);
  pinMode(rightEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncA),  ISR_leftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncA), ISR_rightA, CHANGE);

  Serial.begin(115200);
  Serial.println("Ready!");
#if PRINT_COLUMNS == 6
  Serial.println("# time_s\tvoltage_V\tomegaL\tomegaR\tthetaL\tthetaR");
#else
  Serial.println("# time_s\tvoltage_V\tomegaAvg");
#endif
}

void loop() {
  static unsigned long t0 = millis(), tLast = t0;
  unsigned long now = millis();
  if (now - tLast < desired_Ts_ms) return;
  tLast = now;

  float time_s = (now - t0) / 1000.0f;

  // --- Snapshot counts ---
  long cL, cR;
  noInterrupts();
  cL = leftCount;
  cR = rightCount;
  interrupts();

  // --- Counts -> radians ---
  const float radPerCount = TWO_PI_F / (float)COUNTS_PER_REV;
  float thetaL = (LEFT_SIGN  * (float)cL) * radPerCount;
  float thetaR = (RIGHT_SIGN * (float)cR) * radPerCount;

  // --- Raw velocities from prior theta (exactly per spec) ---
  float omegaL = (thetaL - thetaL_prev) / Ts_s;
  float omegaR = (thetaR - thetaR_prev) / Ts_s;
  thetaL_prev = thetaL;
  thetaR_prev = thetaR;

  // --- Low-pass filter (helps quantization noise) ---
  omegaL_f = a_lp*omegaL_f + (1.0f - a_lp)*omegaL;
  omegaR_f = a_lp*omegaR_f + (1.0f - a_lp)*omegaR;

  // --- Reference schedule: 0 until 1.0 s, then step to OMEGA_REF until 3.0 s ---
  float omegaRef = (time_s >= 1.0f && time_s < 3.0f) ? OMEGA_REF_RAD_S : 0.0f;

  // --- Feedback: use average of both wheels for control ---
  float omegaMeas = 0.5f*(omegaL_f + omegaR_f);
  float e = omegaRef - omegaMeas;

  // --- PI control with simple anti-windup (conditional integration) ---
  // Proposed control voltage (unsat):
  float u_unsat = Kp*e + Ki*integ;
  // Saturate to [0, VMAX] for forward-only demo
  float u = u_unsat;
  if (u > VMAX) u = VMAX;
  if (u < 0.0f) u = 0.0f;

  // Anti-windup: integrate only if not saturating in the wrong direction
  bool saturating_high = (u_unsat > VMAX);
  bool saturating_low  = (u_unsat < 0.0f);
  if ((!saturating_high && !saturating_low) ||
      (saturating_high && e < 0.0f) ||
      (saturating_low  && e > 0.0f)) {
    integ += e * Ts_s;
  }

  // --- Apply same PWM to both wheels ---
  int pwmCmd = (int)((u / BATTERY_VOLTAGE) * 255.0f + 0.5f);
  if (pwmCmd < 0)   pwmCmd = 0;
  if (pwmCmd > 255) pwmCmd = 255;
  analogWrite(leftMpwm,  pwmCmd);
  analogWrite(rightMpwm, pwmCmd);

  // --- Log only first 3 seconds ---
  float voltage = BATTERY_VOLTAGE * ((float)pwmCmd / 255.0f);
  if (time_s <= 3.0f) {
#if PRINT_COLUMNS == 6
    Serial.print(time_s, 3);   Serial.print('\t');
    Serial.print(voltage, 3);  Serial.print('\t');
    Serial.print(omegaL, 6);   Serial.print('\t');  // raw per spec
    Serial.print(omegaR, 6);   Serial.print('\t');  // raw per spec
    Serial.print(thetaL, 6);   Serial.print('\t');
    Serial.println(thetaR, 6);
#else
    float omegaAvg = 0.5f*(omegaL + omegaR); // raw avg
    Serial.print(time_s, 3);   Serial.print('\t');
    Serial.print(voltage, 3);  Serial.print('\t');
    Serial.println(omegaAvg, 6);
#endif
  } else {
    static bool finished = false;
    if (!finished) {
      analogWrite(leftMpwm, 0);
      analogWrite(rightMpwm, 0);
      Serial.println("Finished");
      finished = true;
    }
  }
}