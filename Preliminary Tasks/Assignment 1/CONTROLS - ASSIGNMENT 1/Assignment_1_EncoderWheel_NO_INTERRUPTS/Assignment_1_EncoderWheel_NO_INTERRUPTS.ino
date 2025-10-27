// ---- Pins ----
const byte APIN = 2;   // A / CLK
const byte BPIN = 7;   // B / DT

// Flip to -1 if CW is negative with your wiring
const int8_t DIR_POLARITY = +1;

// Optional tiny debounce for mechanical encoders (us)
const unsigned long CHANGE_DEBOUNCE_US = 800;

int lastA, lastB;
long position = 0;
unsigned long lastChangeUs = 0;

void setup() {
  Serial.begin(9600);
  pinMode(APIN, INPUT_PULLUP);
  pinMode(BPIN, INPUT_PULLUP);
  lastA = digitalRead(APIN);
  lastB = digitalRead(BPIN);
  Serial.println("Polling demo: prints A\\tB on change, pos every 1s");
}

void loop() {
  int thisA = digitalRead(APIN);
  int thisB = digitalRead(BPIN);

  if (thisA != lastA || thisB != lastB) {
    unsigned long t = micros();
    if (t - lastChangeUs >= CHANGE_DEBOUNCE_US) {
      lastChangeUs = t;

      // --- Your print format ---
      Serial.print(thisA); Serial.print('\t'); Serial.println(thisB);

      // --- Count logic (no table) ---
      if (thisA != lastA) position += (thisA == thisB) ? +1 : -1;
      else                position += (thisA != thisB) ? +1 : -1;

      lastA = thisA; lastB = thisB;
    }
  }

  // Timed pos print (1 Hz)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.print("pos = "); Serial.println(position);
    lastPrint = millis();
  }
}