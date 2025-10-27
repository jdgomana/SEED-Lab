// ---- Pins ----
const byte APIN = 2;   // A / CLK (INT0)
const byte BPIN = 7;   // B / DT

// CW positive (set to -1 if reversed)
const int8_t DIR_POLARITY = +1;

// Assignment-style debounce (ms). Use 1–2 ms for real use.
const unsigned long ISR_DEBOUNCE_MS = 50;

volatile int  lastA, lastB;
volatile long position = 0;
volatile unsigned long lastIsrMs = 0;

void onAChange() {
  unsigned long now = millis();
  if (now - lastIsrMs < ISR_DEBOUNCE_MS) return;
  lastIsrMs = now;

  int thisA = digitalRead(APIN);
  int thisB = digitalRead(BPIN);

  // --- Your print format (A\tB on each change) ---
  if (thisA != lastA || thisB != lastB) {
    Serial.print(thisA); Serial.print('\t'); Serial.println(thisB);

    // --- Count logic (no table) ---
    if (thisA != lastA) position += ((thisA == thisB) ? +1 : -1) * DIR_POLARITY;
    else                position += ((thisA != thisB) ? +1 : -1) * DIR_POLARITY;

    lastA = thisA; lastB = thisB;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(APIN, INPUT_PULLUP);
  pinMode(BPIN, INPUT_PULLUP);
  lastA = digitalRead(APIN);
  lastB = digitalRead(BPIN);

  attachInterrupt(digitalPinToInterrupt(APIN), onAChange, CHANGE);
  Serial.println("Interrupt demo: prints A\\tB on change, pos every 1s (ISR debounced)");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    noInterrupts(); long p = position; interrupts();
    Serial.print("pos = "); Serial.println(p);
    lastPrint = millis();
  }
}