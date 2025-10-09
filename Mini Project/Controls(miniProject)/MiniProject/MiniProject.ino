#include <Wire.h>

/* ---------------- Pins (UNO) ---------------- */
#define MY_ADDR 8               // I2C addr (A4=SDA, A5=SCL)

const int EN       = 4;         // motor driver enable (HIGH = on)
const int MDIR[2]  = {7, 8};    // left, right direction pins
const int MPWM[2]  = {9, 10};   // left, right PWM pins

const int ENC_A[2] = {2, 3};    // left A (INT0), right A (INT1)
const int ENC_B[2] = {5, 6};    // left B, right B

/* -------------- Geometry / scaling ---------- */
long  CPR[2]       = {3200, 3200};               // set to your measured CPRs
float radPerCnt[2] = {0,0};

// Encoder direction (flip if a wheel counts “backward” vs physical rotation)
int ENC_SIGN[2]    = {-1, +1};

// Motor polarity (flip if motor spins the wrong direction for positive voltage)
int MOTOR_SIGN[2]  = {-1, +1};

/* -------------- Loop timing ----------------- */
const unsigned long Ts_ms = 10;                  // 100 Hz
const float Ts_s = Ts_ms/1000.0f;

/* -------------- Inner (velocity) PI --------- */
float Kp[2] = {6.0f, 6.0f};
float Ki[2] = {0.0f,0.0};
float integVel[2] = {0,0};
float VEL_CAP = 10000.0f;                            // speed cap (rad/s)

/* -------------- Outer (position) PI (PER WHEEL) --------- */
// Edit these independently: {LEFT, RIGHT}
float Kp_pos[2] = {2.0f, 2.0f};   // was single 0.55
float Ki_pos[2] = {0.6f, 0.6f};   // was single 0.25f
float integPos[2] = {0,0};

// Example per-wheel tuning (uncomment to try):
// float Kp_pos[2] = {0.60f, 0.35f};
// float Ki_pos[2] = {0.25f, 0.18f};

// HOLD/hysteresis to prevent re-start when touched
const float POS_TOL_HOLD = 0.02f;                // ~1.1° window to “enter hold”
const float POS_TOL_EXIT = 0.06f;                // ~3.4° to “exit hold”
const float VEL_TOL      = 0.10f;                // rad/s
bool hold[2] = {false,false};

/* -------------- Electrical ------------------ */
const float VBATT = 7.5f;
const float VMAX  = 0.95f*VBATT;
const float U_DEADBAND = 0.8f;                  // ignore tiny commands

/* -------------- Encoder state --------------- */
volatile long cnt[2] = {0,0};
static   long cnt_prev[2] = {0,0};
float theta[2] = {0,0};                           // cumulative radians
float omega_f[2] = {0,0};                         // filtered rad/s
const float fc_hz = 15.0f;
const float a_lp  = expf(-2.0f*3.14159265358979f*fc_hz*Ts_s);

/* -------------- Desired targets ------------- */
float desired_pos[2]   = {0,0};                   // radians (absolute)
float desired_speed[2] = {0,0};                   // rad/s from position PI

/* -------------- Helpers --------------------- */
static inline int V2PWM(float u){
  int p = (int)((fabs(u)/VBATT)*255.0f + 0.5f);
  if (p<0) p=0; if (p>255) p=255; return p;
}
static inline void driveSigned(int i, float u){
  // Apply motor polarity; choose DIR by sign, PWM by |u|
  float u_eff = MOTOR_SIGN[i] * u;
  if (u_eff >= 0.0f) digitalWrite(MDIR[i], HIGH);
  else               digitalWrite(MDIR[i], LOW);
  if (fabs(u_eff) < U_DEADBAND) analogWrite(MPWM[i], 0);
  else                          analogWrite(MPWM[i], V2PWM(u_eff));
}
static inline void stopMotor(int i){ analogWrite(MPWM[i], 0); }

/* -------------- ISRs ------------------------ */
void ISR_leftA(){  int A=digitalRead(ENC_A[0]), B=digitalRead(ENC_B[0]);
                   cnt[0] += ENC_SIGN[0] * ((A==B)? +1 : -1); }
void ISR_rightA(){ int A=digitalRead(ENC_A[1]), B=digitalRead(ENC_B[1]);
                   cnt[1] += ENC_SIGN[1] * ((A==B)? +1 : -1); }

/* -------------- I2C: add +π to setpoints ---- */
void onRx(int){
  char d0=0,d1=0;
  while (Wire.available()){ char c=(char)Wire.read(); d0=d1; d1=c; }
  if ((d0=='0'||d0=='1') && (d1=='0'||d1=='1')){
    Serial.print("Goal Position: "); Serial.print(d0); Serial.println(d1);
    if (d0=='1') { desired_pos[0] += 3.14159265358979f/2; hold[0] = false; }
    if (d1=='1') { desired_pos[1] += 3.14159265358979f/2; hold[1] = false; }
  }
}

/* -------------- Setup ----------------------- */
void setup(){
  pinMode(EN, OUTPUT); digitalWrite(EN, HIGH);
  for(int i=0;i<2;i++){
    pinMode(MDIR[i], OUTPUT);  digitalWrite(MDIR[i], HIGH);
    pinMode(MPWM[i], OUTPUT);  analogWrite(MPWM[i], 0);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), ISR_leftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), ISR_rightA, CHANGE);

  for(int i=0;i<2;i++) radPerCnt[i] = 2.0f*3.14159265358979f/(float)CPR[i];

  Wire.begin(MY_ADDR);
  Wire.onReceive(onRx);

  Serial.begin(115200);
  Serial.println("# voltage_V\tleft_theta\tright_theta");
}

/* -------------- Main loop ------------------- */
void loop(){
  static unsigned long tLast = millis();
  unsigned long now = millis();
  if (now - tLast < Ts_ms) return;
  tLast = now;

  // Snapshot counts
  long c[2]; noInterrupts(); c[0]=cnt[0]; c[1]=cnt[1]; interrupts();

  // Counts → theta, omega (filtered, signed)
  long dC[2] = { c[0]-cnt_prev[0], c[1]-cnt_prev[1] };
  cnt_prev[0]=c[0]; cnt_prev[1]=c[1];

  for(int i=0;i<2;i++){
    theta[i] += dC[i]*radPerCnt[i];
    float omega = (dC[i]*radPerCnt[i])/Ts_s;
    omega_f[i] = a_lp*omega_f[i] + (1.0f-a_lp)*omega;
  }

  // --------- Outer PI (position → desired_speed) with HOLD hysteresis ----------
  for(int i=0;i<2;i++){
    float e_pos = desired_pos[i] - theta[i];

    if (hold[i]) {
      // Stay in hold unless error grows beyond EXIT threshold
      if (fabs(e_pos) > POS_TOL_EXIT) {
        hold[i] = false;  // re-engage control
      } else {
        desired_speed[i] = 0.0f;
        integPos[i] = 0.0f;      // freeze/bleed integral in hold
        continue;                // skip PI for this wheel
      }
    }

    // Not in hold: run PI (PER-WHEEL GAINS)
    integPos[i] += e_pos * Ts_s;
    if (integPos[i] > 3.0f) integPos[i] = 3.0f;
    if (integPos[i] < -3.0f) integPos[i] = -3.0f;

    desired_speed[i] = Kp_pos[i]*e_pos + Ki_pos[i]*integPos[i];

    // limit desired speed (global cap, no taper)
    if (desired_speed[i] >  VEL_CAP) desired_speed[i] =  VEL_CAP;
    if (desired_speed[i] < -VEL_CAP) desired_speed[i] = -VEL_CAP;

    // If we're essentially at target and not moving, enter hold
    if (fabs(e_pos) < POS_TOL_HOLD && fabs(omega_f[i]) < VEL_TOL) {
      hold[i] = true;
      desired_speed[i] = 0.0f;
      integPos[i] = 0.0f;
      desired_pos[i] = theta[i];  // snap desired to current
    }
  }

  // --------- Inner PI (velocity → signed voltage) --------
  float uV[2] = {0,0};
  for(int i=0;i<2;i++){
    float e_vel  = desired_speed[i] - omega_f[i];     // signed error
    float u_uns  = Kp[i]*e_vel + Ki[i]*integVel[i];   // signed volts
    float u      = u_uns;
    if (u >  VMAX) u =  VMAX;
    if (u < -VMAX) u = -VMAX;

    // anti-windup
    bool satH = (u_uns >  VMAX), satL = (u_uns < -VMAX);
    if ((!satH && !satL) || (satH && e_vel<0) || (satL && e_vel>0))
      integVel[i] += e_vel*Ts_s;

    uV[i] = u;

    if (hold[i]) stopMotor(i);
    else         driveSigned(i, u);
  }

  // Telemetry: avg |voltage|, absolute left/right radians
  float vavg = 0.5f*(fabs(uV[0]) + fabs(uV[1]));
  Serial.print(vavg,3); Serial.print('\t');
  Serial.print(theta[0],5); Serial.print('\t');
  Serial.println(theta[1],5);
  delay(1);
}
