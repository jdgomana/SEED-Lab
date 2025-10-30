#include <Wire.h>
#include <math.h>

/* =========================================================
   HARDWARE + WIRING
   ---------------------------------------------------------
   - We run on an Arduino UNO as I2C slave (addr = 8)
   - You have your motor driver enable on D4
   - Your left/right motor DIR/PWM pins are SWAPPED in wiring,
     so we keep that mapping here
   ========================================================= */
#define MY_ADDR 8
const int EN       = 4;          // motor driver enable (HIGH = on)

/* Make index 0 = LEFT, index 1 = RIGHT (your wiring is swapped) */
const int MDIR[2]  = {8, 7};     // MDIR[0]=pin 8 → LEFT DIR, MDIR[1]=pin 7 → RIGHT DIR
const int MPWM[2]  = {10, 9};    // MPWM[0]=pin 10 → LEFT PWM, MPWM[1]=pin 9 → RIGHT PWM

/* Encoders (A = interrupt pins, B = regular digital) */
const int ENC_A[2] = {2, 3};     // ENC_A[0]=2 → LEFT encoder A (INT0), ENC_A[1]=3 → RIGHT encoder A (INT1)
const int ENC_B[2] = {5, 6};     // ENC_B[0]=5 → LEFT encoder B, ENC_B[1]=6 → RIGHT encoder B
const int LED_PIN  = LED_BUILTIN;

/* =========================================================
   GEOMETRY / SCALING
   ---------------------------------------------------------
   - CPR[]: counts per revolution for each encoder
   - radPerCnt[]: how many radians per 1 encoder count
   - ENC_SIGN[]: fixes encoder direction
   - MOTOR_SIGN[]: fixes actual motor polarity vs “forward” concept
   ========================================================= */
long  CPR[2]       = {3200, 3200};     // measured CPR for left/right wheel
float radPerCnt[2] = {0, 0};           // filled in setup: 2*pi / CPR

// Signs: forward spin should make counts/angles increase
volatile int ENC_SIGN[2]  = {+1, -1};  // LEFT counts positive, RIGHT counts negative → fixes your physical encoder wiring
int MOTOR_SIGN[2]         = {-1, +1};  // LEFT motor needs inverted voltage, RIGHT motor normal

// Robot geometry (used for odometry and body→wheel mapping)
const float R_wheel = 0.0762f;   // wheel radius [m]   (3.0 in)
const float L_axle  = 0.3505f;   // axle length [m]    (13.8 in)

/* =========================================================
   ODOMETRY CALIBRATION
   ---------------------------------------------------------
   - DIST_K multiplies the *average* wheel motion → distance
   - YAW_K multiplies wheel difference → heading
   - You tuned these to get “real floor” numbers
   ========================================================= */
float DIST_K = 1.93f;            // distance scale — tuned from your tape tests
const float YAW_K  = 1.78f;      // yaw scale — tuned from your 180° turns

/* =========================================================
   LOOP TIMING
   ========================================================= */
const unsigned long Ts_ms = 10;     // run control at 100 Hz
const float Ts_s = Ts_ms / 1000.0f; // seconds version

/* =========================================================
   INNER (WHEEL) VELOCITY PI
   ---------------------------------------------------------
   - This is the fast loop: desired_speed[i] → motor volts
   - One PI per wheel
   - This loop is what actually drives the motors
   ========================================================= */
float Kp[2]    = {12.0f, 12.0f};   // per-wheel velocity P gain
float Ki[2]    = {0.0f,  0.0f};    // per-wheel velocity I gain (off for now)
float integVel[2] = {0, 0};        // per-wheel velocity integral state

/* =========================================================
   FORWARD DISTANCE PI (ROBOT LEVEL)
   ---------------------------------------------------------
   - This is the “go forward X meters/feet” controller
   - It outputs a *body* forward speed vp_cmd (m/s)
   ========================================================= */
float Kp_p_o   = 1.0f;             // distance P
float Ki_p_o   = 0.10f;            // distance I (small to avoid shove)
float ep_int   = 0.0f;             // distance integrator

/* =========================================================
   ELECTRICAL / OUTPUT LIMITS
   ========================================================= */
const float VBATT = 7.5f;          // assumed battery / driver usable voltage
const float VMAX  = VBATT;         // cap wheel PI at full battery
const float U_DEADBAND = 0.05f;    // ignore tiny volts → avoids buzz

/* =========================================================
   PER-WHEEL RUN PARAMS
   ---------------------------------------------------------
   - WGAIN[] lets you slow one wheel slightly if it always wins
   - U_MIN_RUN[] is “minimum shove” to break static friction
   - U_BIAS[] is tiny offset to help a slower wheel
   ========================================================= */
float WGAIN[2]      = {1.0f, 1.0f};     // 1.0 for both → no extra scaling
float U_MIN_RUN[2]  = {0.62f, 0.62f};   // both wheels need about this to start
float U_BIAS[2]     = {+0.02f, 0.00f};  // left gets +0.02 V to keep up a bit

/* =========================================================
   ANTI-CHATTER / SLEW
   ---------------------------------------------------------
   - u_cmd[] is the *slewed* voltage we actually send
   - we limit the change per loop to avoid brownouts
   ========================================================= */
float u_cmd[2] = {0,0};
const float DU_MAX    = 0.25f;      // max |ΔU| per 10 ms
const float SPEED_EPS = 0.15f;      // small speed threshold for sign logic

/* =========================================================
   ENCODER STATE
   ---------------------------------------------------------
   - cnt[] updated in ISRs
   - we keep previous count to get dC
   - omega_f[] is lowpass filtered wheel speed
   ========================================================= */
volatile long cnt[2] = {0, 0};      // live counts from ISRs
static long  cnt_prev[2] = {0, 0};  // last-loop counts
float theta[2] = {0, 0};            // cumulative wheel angles [rad]
float omega_f[2] = {0, 0};          // filtered wheel ang. vel. [rad/s]
const float fc_hz = 20.0f;          // cutoff for wheel speed filter
const float a_lp = expf(-(2.0f * PI) * fc_hz * Ts_s);  // 1st-order coeff

/* =========================================================
   DESIRED TARGETS (PER WHEEL)
   ---------------------------------------------------------
   - robot controllers create a *body* cmd → we convert to
     desired wheel angular speeds here
   ========================================================= */
float desired_speed[2] = {0, 0};

/* =========================================================
   ROBOT POSE (ODOMETRY STATE)
   ---------------------------------------------------------
   - p_m  = total distance traveled (m)
   - phi_r = robot heading (rad)
   ========================================================= */
float p_m  = 0.0f;
float phi_r = 0.0f;

/* =========================================================
   HEADING CONTROL (SPLIT GAINS)
   ---------------------------------------------------------
   We run TWO heading controllers:
   1) TURN controller — used in TURN phase, sharper
   2) FORWARD controller — used when driving straight, softer
   ========================================================= */
// TURN (snappier)
float Kp_phi_turn = 1.15f;
float Ki_phi_turn = 0.22f;
float Kd_phi_turn = 2.00f;
float ephi_int_turn = 0.0f;     // TURN integral state

// FORWARD (softer PD)
float Kp_phi_fwd  = 0.90f;
float Ki_phi_fwd  = 0.00f;      // no I while driving forward → avoids slow weave
float Kd_phi_fwd  = 1.80f;
float ephi_int_fwd = 0.0f;      // FORWARD integral (unused right now)

const float EPHI_INT_MAX = 0.4f;  // clamp for heading I

/* =========================================================
   BODY-LEVEL COMMAND LIMITS
   ---------------------------------------------------------
   - vp_cmd: forward linear speed (m/s)
   - w_cmd : yaw rate (rad/s)
   - different caps for TURN vs FORWARD
   ========================================================= */
float vp_cmd = 0.0f;
float w_cmd  = 0.0f;
const float VP_MAX      = 0.55f;  // forward speed cap
const float W_MAX_FWD   = 0.40f;  // steering cap during FORWARD
const float W_MAX_TURN  = 0.55f;  // bigger cap for pure TURN

/* =========================================================
   QUIET WINDOWS
   ---------------------------------------------------------
   - make tiny commands = 0 → keeps robot still at end
   ========================================================= */
const float W_QUIET  = 0.015f;
const float VP_QUIET = 0.02f;

/* =========================================================
   MOTION PROFILE
   ---------------------------------------------------------
   - used to limit forward speed based on remaining distance
   - A_BRAKE is the assumed decel
   ========================================================= */
const float A_BRAKE = 1.10f;

/* =========================================================
   MISSION INPUTS (you set these before upload)
   ---------------------------------------------------------
   - angle in deg
   - distance in ft → auto to meters (5 decimals)
   ========================================================= */
float DESIRED_ANGLE_DEG = 180.0f;
float DESIRED_DIST_FT   = 2.0f;
float DESIRED_DIST_M    = 0.0f;

/* =========================================================
   PHASE MACHINE
   ---------------------------------------------------------
   PREP → TURN → FORWARD → ALIGN → DONE
   - PREP: wait 4s, snapshot current pose, build targets
   - TURN: rotate to phi_des
   - FORWARD: move to p_des
   - ALIGN: final little twist so wheels “land” even
   - DONE: motors quiet
   ========================================================= */
enum Phase { PREP, TURN, FORWARD, ALIGN, DONE };
Phase phase = PREP;

unsigned long boot_ms, turnStartMs, moveStartMs, alignStartMs;
const unsigned long ARMING_DURATION_MS = 4000UL;  // wait 4s after power
const unsigned long PHASE_TIMEOUT_MS   = 6000UL;  // each phase max 6s

float phi_des = 0.0f;              // absolute heading target, set in PREP
float p_des   = 0.0f;              // absolute distance target, set in PREP
float p_at_forward_start = 0.0f;   // where we started FORWARD (for distance check)
float e_p_dbg = 0.0f;              // just to share dist error with inner loop

/* =========================================================
   TURN DWELL
   ---------------------------------------------------------
   - TURN should be “done and still” for some ms
   ========================================================= */
const float PHI_TOL_TURN = 0.35f * PI/180.0f;  // ≈0.35°
const float W_TOL_TURN   = 0.05f;              // rad/s
const unsigned long TURN_SETTLE_MS = 500UL;    // 0.5 s of clean turn
unsigned long turn_ok_since = 0;

/* =========================================================
   ALIGN PHASE PARAMS
   ---------------------------------------------------------
   - after forward we compute how much farther R went than L
   - then we spin in place to compensate that
   ========================================================= */
float phi_align_des = 0.0f;                // heading to trim to
float theta_start_forward[2] = {0,0};      // wheel angles when FORWARD started
const float ALIGN_TOL_ANG = 0.35f * PI/180.0f;
const float ALIGN_TOL_W   = 0.04f;
const unsigned long ALIGN_MAX_MS = 700UL;
const float W_MAX_ALIGN = 0.35f;

/* =========================================================
   WHEEL MATCH ACCUMULATORS
   ---------------------------------------------------------
   - while driving straight we average left/right speeds
   - at the end we can print a suggestion for WGAIN[]
   ========================================================= */
double sum_wL = 0.0, sum_wR = 0.0;
unsigned long n_w = 0;

/* =========================================================
   HELPERS
   ========================================================= */
static inline float ft2m(float ft){ return ft * 0.3048f; }
static inline float round5(float x){ return floorf(x*100000.0f+0.5f)/100000.0f; }

static inline int V2PWM(float u){
  int p = (int)((fabs(u)/VBATT)*255.0f+0.5f);
  if (p<0) p=0; if (p>255) p=255; return p;
}

static inline void driveSigned(int i, float u){
  // apply motor polarity
  float u_eff = MOTOR_SIGN[i]*u;
  digitalWrite(MDIR[i], (u_eff>=0.0f)?HIGH:LOW);
  if (fabs(u_eff)<U_DEADBAND) analogWrite(MPWM[i], 0);
  else                        analogWrite(MPWM[i], V2PWM(u_eff));
}

/* =========================================================
   ENCODER ISRs
   ---------------------------------------------------------
   - very small: just compute +1 / -1 using A/B phase and ENC_SIGN
   ========================================================= */
void ISR_leftA(){ int A=digitalRead(ENC_A[0]); int B=digitalRead(ENC_B[0]); cnt[0]+=ENC_SIGN[0]*((A!=B)?+1:-1); }
void ISR_rightA(){int A=digitalRead(ENC_A[1]); int B=digitalRead(ENC_B[1]); cnt[1]+=ENC_SIGN[1]*((A!=B)?+1:-1); }

/* =========================================================
   SETUP
   ---------------------------------------------------------
   - set pins, interrupts
   - compute radPerCnt[]
   - start serial
   ========================================================= */
void setup(){
  pinMode(EN,OUTPUT); digitalWrite(EN,HIGH);
  for(int i=0;i<2;i++){
    pinMode(MDIR[i],OUTPUT); pinMode(MPWM[i],OUTPUT);
    pinMode(ENC_A[i],INPUT_PULLUP);
    pinMode(ENC_B[i],INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), ISR_leftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), ISR_rightA, CHANGE);

  for(int i=0;i<2;i++) radPerCnt[i] = (2.0f*PI)/CPR[i];

  Serial.begin(115200);
  Serial.println(F("# thetaL(rad)\tthetaR(rad)\tp_m(m)\tphi_r(rad)\tcntL\tcntR\tAL/BL\tAR/BR"));
  boot_ms = millis();
}

/* =========================================================
   MAIN LOOP (100 Hz)
   ========================================================= */
void loop(){
  static unsigned long tLast=millis(); unsigned long now=millis();
  if(now-tLast < Ts_ms) return; tLast=now;

  /* ---------- 1) READ ENCODERS AND COMPUTE WHEEL SPEEDS --------- */
  long c[2]; noInterrupts(); c[0]=cnt[0]; c[1]=cnt[1]; interrupts();
  long dC[2]={c[0]-cnt_prev[0],c[1]-cnt_prev[1]};
  cnt_prev[0]=c[0]; cnt_prev[1]=c[1];

  float dtheta[2];
  for(int i=0;i<2;i++){
    dtheta[i]=dC[i]*radPerCnt[i];        // counts → radians
    theta[i]+=dtheta[i];                 // accumulate angle
    float omega = dtheta[i]/Ts_s;        // raw wheel speed
    omega_f[i] = a_lp*omega_f[i]+(1.0f-a_lp)*omega;  // lowpass
  }

  /* ---------- 2) ODOMETRY: robot distance + heading -------------- */
  float dp   = DIST_K*(0.5f*R_wheel*(dtheta[0]+dtheta[1]));       // avg wheel motion → distance
  float dphi = YAW_K*((R_wheel/L_axle)*(dtheta[1]-dtheta[0]));    // wheel diff → heading
  p_m  += dp;
  phi_r += dphi;

  /* ---------- 3) ARM / INITIALIZE MISSION ------------------------ */
  if(phase==PREP && (now-boot_ms)>=ARMING_DURATION_MS){
    DESIRED_DIST_M = round5(ft2m(DESIRED_DIST_FT));    // ft→m (5 decimals)
    Serial.print(F("## RESOLVED DIST: "));
    Serial.print(DESIRED_DIST_FT,3); Serial.print(F(" ft -> "));
    Serial.print(DESIRED_DIST_M,5);  Serial.println(F(" m"));

    // build absolute targets off current pose
    phi_des = phi_r + DESIRED_ANGLE_DEG*(PI/180.0f);
    p_des   = p_m  + DESIRED_DIST_M;
    ephi_int_turn = ephi_int_fwd = 0.0f;
    ep_int = 0.0f;

    turnStartMs = now;
    moveStartMs = now;
    turn_ok_since = 0;

    // pick phase
    phase = (fabs(DESIRED_ANGLE_DEG)>1e-3f)?TURN:FORWARD;
    sum_wL=sum_wR=0.0; n_w=0;
  }

  /* ---------- 4) COMMON THRESHOLDS ------------------------------- */
  const float P_TOL   = 0.0075f;  // for forward finish
  const float V_TOL   = 0.03f;    // for forward finish

  float phi_dot_meas = YAW_K*((R_wheel/L_axle)*(omega_f[1]-omega_f[0])); // yaw rate from wheels
  float rho_dot_meas = DIST_K*(0.5f*R_wheel*(omega_f[0]+omega_f[1]));    // forward speed

  vp_cmd=w_cmd=0.0f; // default: no motion unless phase sets it

  /* =====================================================
     PHASE: TURN  (robot-level angle controller)
     ===================================================== */
  if(phase==TURN){
    // heading error
    float e_phi = phi_des-phi_r;
    while(e_phi>PI) e_phi-=2*PI;
    while(e_phi<-PI) e_phi+=2*PI;

    // TURN PI(D)
    ephi_int_turn += e_phi*Ts_s;
    if(ephi_int_turn> EPHI_INT_MAX) ephi_int_turn= EPHI_INT_MAX;
    if(ephi_int_turn<-EPHI_INT_MAX) ephi_int_turn=-EPHI_INT_MAX;

    w_cmd = Kp_phi_turn*e_phi + Ki_phi_turn*ephi_int_turn - Kd_phi_turn*phi_dot_meas;

    // clamp + quiet
    if(w_cmd> W_MAX_TURN) w_cmd= W_MAX_TURN;
    if(w_cmd<-W_MAX_TURN) w_cmd=-W_MAX_TURN;
    if(fabs(w_cmd)<W_QUIET) w_cmd=0.0f;

    // check if turn is good AND stable
    bool turn_ok = (fabs(e_phi) < PHI_TOL_TURN) && (fabs(phi_dot_meas) < W_TOL_TURN);
    if (turn_ok) {
      if (turn_ok_since == 0) turn_ok_since = now;  // start dwell timer
    } else {
      turn_ok_since = 0;
    }
    bool dwell_satisfied = (turn_ok_since != 0) && ((now - turn_ok_since) >= TURN_SETTLE_MS);

    // exit TURN → go FORWARD
    if(dwell_satisfied || (now-turnStartMs)>PHASE_TIMEOUT_MS){
      if(fabs(DESIRED_DIST_M)>1e-3f){
        phase=FORWARD; moveStartMs=now; p_at_forward_start=p_m;
        theta_start_forward[0]=theta[0];
        theta_start_forward[1]=theta[1];
        sum_wL=sum_wR=0.0; n_w=0;
      } else { phase=DONE; }
      // reset forward heading I
      ephi_int_fwd = 0.0f;
    }
  }

  /* =====================================================
     PHASE: FORWARD (robot-level distance controller)
     ===================================================== */
  else if(phase==FORWARD){
    float e_p = p_des - p_m;   // distance error
    e_p_dbg = e_p;             // for near-stop logic later

    // keep you pointed at phi_des while moving
    float e_phi = phi_des - phi_r;
    while(e_phi>PI) e_phi-=2*PI;
    while(e_phi<-PI) e_phi+=2*PI;

    // distance PI
    ep_int += e_p*Ts_s;
    if(ep_int>0.40f) ep_int=0.40f;
    if(ep_int<-0.40f) ep_int=-0.40f;

    float vp_cmd_raw = Kp_p_o*e_p + Ki_p_o*ep_int;

    // motion profile: limit max speed so we can stop
    float v_stop = sqrtf(fmaxf(0.0f, 2.0f*A_BRAKE*fabsf(e_p)));
    float vp_lim = fminf(VP_MAX, v_stop);
    vp_cmd = fmaxf(-vp_lim, fminf(vp_cmd_raw, vp_lim));

    // heading PD during forward, scaled
    float speed_scale = fabs(vp_cmd)/VP_MAX;   // 0..1
    float w_scale     = 0.30f + 0.70f*speed_scale;
    w_cmd  = (Kp_phi_fwd*e_phi + Ki_phi_fwd*ephi_int_fwd - Kd_phi_fwd*phi_dot_meas) * w_scale;

    // clamp + quiet
    if(w_cmd> W_MAX_FWD) w_cmd= W_MAX_FWD;
    if(w_cmd<-W_MAX_FWD) w_cmd=-W_MAX_FWD;
    if(fabs(vp_cmd)<VP_QUIET) vp_cmd=0.0f;
    if(fabs(w_cmd) <W_QUIET ) w_cmd =0.0f;

    // gather wheel-speed stats for WGAIN suggestions
    if (fabs(w_cmd) < 0.05f && fabs(vp_cmd) > 0.05f) {
      sum_wL += fabs(omega_f[0]);
      sum_wR += fabs(omega_f[1]);
      n_w++;
    }

    // finish FORWARD → go ALIGN
    if((fabs(e_p)<P_TOL && fabs(rho_dot_meas)<V_TOL) || (now-moveStartMs)>PHASE_TIMEOUT_MS){
      // how far each wheel actually went during FORWARD
      float sL = DIST_K * R_wheel * (theta[0] - theta_start_forward[0]);
      float sR = DIST_K * R_wheel * (theta[1] - theta_start_forward[1]);
      float dS = sR - sL; // >0 → right actually traveled farther

      // small twist to equalize: Δφ = -dS / L
      phi_align_des = phi_r - (dS / L_axle);

      phase = ALIGN;
      alignStartMs = now;
      ephi_int_turn = 0.0f;  // reuse turn integrator here
      vp_cmd = w_cmd = 0.0f;
    }
  }

  /* =====================================================
     PHASE: ALIGN (final little twist to line up wheels)
     ===================================================== */
  else if (phase==ALIGN){
    float e_phi = phi_align_des - phi_r;
    while(e_phi>PI) e_phi-=2*PI;
    while(e_phi<-PI) e_phi+=2*PI;

    ephi_int_turn += e_phi*Ts_s;
    if(ephi_int_turn> EPHI_INT_MAX) ephi_int_turn= EPHI_INT_MAX;
    if(ephi_int_turn<-EPHI_INT_MAX) ephi_int_turn=-EPHI_INT_MAX;

    vp_cmd = 0.0f;  // no translation
    float phi_dot_meas_local = YAW_K*((R_wheel/L_axle)*(omega_f[1]-omega_f[0]));
    w_cmd  = Kp_phi_turn*e_phi + Ki_phi_turn*ephi_int_turn - Kd_phi_turn*phi_dot_meas_local;

    if(w_cmd>  W_MAX_ALIGN) w_cmd=  W_MAX_ALIGN;
    if(w_cmd< -W_MAX_ALIGN) w_cmd=-W_MAX_ALIGN;
    if(fabs(w_cmd)<W_QUIET)  w_cmd=0.0f;

    bool angle_ok = (fabs(e_phi) < ALIGN_TOL_ANG);
    bool spin_ok  = (fabs(phi_dot_meas_local) < ALIGN_TOL_W);
    bool timeup   = (now - alignStartMs) > ALIGN_MAX_MS;

    if ((angle_ok && spin_ok) || timeup){
      phase = DONE;
      vp_cmd = w_cmd = 0.0f;
      ep_int = ephi_int_turn = ephi_int_fwd = 0.0f;
    }
  }

  /* =====================================================
     PHASE: DONE
     ===================================================== */
  else { vp_cmd=w_cmd=0.0f; }

  /* =====================================================
     5) BODY CMDS → WHEEL SPEED SETPOINTS
     ===================================================== */
  float vL = vp_cmd - 0.5f*L_axle*w_cmd;  // left linear speed
  float vR = vp_cmd + 0.5f*L_axle*w_cmd;  // right linear speed
  desired_speed[0] = (vL / R_wheel) * WGAIN[0];  // rad/s
  desired_speed[1] = (vR / R_wheel) * WGAIN[1];  // rad/s

  /* =====================================================
     6) INNER VELOCITY PI (per wheel)
     ===================================================== */
  bool near_stop = (phase==FORWARD && fabs(e_p_dbg)<0.12f); // allow tiny reverse when stopping
  for(int i=0;i<2;i++){
    float e_vel = desired_speed[i]-omega_f[i];
    float u = Kp[i]*e_vel + Ki[i]*integVel[i] + U_BIAS[i];

    if(fabs(u)<VMAX) integVel[i]+=e_vel*Ts_s;
    if(u> VMAX) u= VMAX; if(u<-VMAX) u=-VMAX;

    // single-sided except when braking near stop
    if(!near_stop){
      if(desired_speed[i]>SPEED_EPS && u<0) u=0;
      if(desired_speed[i]<-SPEED_EPS && u>0) u=0;
    }

    // stiction kick
    if(u!=0 && fabs(u)<U_MIN_RUN[i]) u=(u>0)?U_MIN_RUN[i]:-U_MIN_RUN[i];

    // slew
    float du=u-u_cmd[i];
    if(du> DU_MAX) du= DU_MAX; if(du<-DU_MAX) du=-DU_MAX;
    u_cmd[i]+=du;

    driveSigned(i,u_cmd[i]);
  }

  /* =====================================================
     7) TELEMETRY (every 50 ms)
     ===================================================== */
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 50) {
    lastPrint = now;
    int aL = digitalRead(ENC_A[0]), bL = digitalRead(ENC_B[0]);
    int aR = digitalRead(ENC_A[1]), bR = digitalRead(ENC_B[1]);

    Serial.print(theta[0],5); Serial.print('\t');
    Serial.print(theta[1],5); Serial.print('\t');
    Serial.print(p_m,5);      Serial.print('\t');
    Serial.print(phi_r,5);    Serial.print('\t');
    Serial.print((long)cnt[0]); Serial.print('\t');
    Serial.print((long)cnt[1]); Serial.print('\t');

    // AL/BL
    Serial.print(aL); Serial.print(aL ? "H" : "L"); Serial.print('/');
    Serial.print(bL); Serial.print(bL ? "H" : "L"); Serial.print('\t');

    // AR/BR
    Serial.print(aR); Serial.print(aR ? "H" : "L"); Serial.print('/');
    Serial.println(bR ? "1H" : "0L");
  }
}
