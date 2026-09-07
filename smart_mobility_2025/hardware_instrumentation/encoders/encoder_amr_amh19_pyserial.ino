// ======================================================================
//  AMH19 - ESP32 Encoder to JSON Interface
// ----------------------------------------------------------------------
//  Author: Abraham Moro-Hernandez (AMH19)
//
//  Description:
//      Firmware to read the encoder using interrupts
//      on the ESP32 and stream measurements via JSON at 50 ms.
//      This script provides:
//
//          • Raw pulse count  (variable 'contador')
//          • Distance traveled (meters) using calibrated K_DIST
//          • Linear velocity  (m/s)
//
//  Notes:
//      - Calibration constant K_DIST corresponds to AMR1 wheel:
//            10200 pulses per wheel revolution (average obtained after tests)
//            K_DIST = meters_per_pulse
//
//      - JSON output format is compatible with ROS2 node:
//            {"pulsos":1234, "dist":0.5678, "vel":0.1234}
//
//      - Interrupt-driven quadrature ensures accurate readings,
//        even at high wheel speeds.
// ======================================================================


// ======================
//  PIN DEFINITIONS
// ======================
#define ENCODER_A 34    // Quadrature channel A
#define ENCODER_B 35    // Quadrature channel B
#define ENCODER_Z 32    // Index Z (not used here but initialized)


// ======================
//  GLOBAL VARIABLES
// ======================
volatile long contador = 0;   // Pulse accumulator
volatile int  lastA = 0;      // Stores last state of channel A

// Calibration constant (meters per pulse)
//  Based on experimentally measured 10200 pulses per wheel revolution.
float K_DIST = 0.000174;


// ==============================
//  ISR: Quadrature Decode (A)
// ==============================
//  Determines pulse direction based on A/B phase relation
void IRAM_ATTR encoderA_ISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  if (A != lastA) {
    if (A == B) contador++;    // forward rotation
    else        contador--;    // reverse rotation
  }
  lastA = A;
}


// ==============================================
//  ISR: Index Z (currently unused but available)
// ==============================================
void IRAM_ATTR encoderZ_ISR() {}


// ========
//  SETUP
// ========
void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), encoderZ_ISR, RISING);

  Serial.println("ESP32 Encoder Online");
}


// ===============================
//  MAIN LOOP (20 Hz JSON Output)
// ===============================
long lastCount = 0;
unsigned long lastTime = 0;

void loop() {
  unsigned long now = millis();

  // Publish data every 50 ms (20 Hz)
  if (now - lastTime >= 50) {

    long current = contador;
    long delta = current - lastCount;
    lastCount = current;

    float dt = (now - lastTime) / 1000.0;

    // -------------------------------
    //  Compute distance and velocity
    // -------------------------------
    float dist_m = current * K_DIST;            // cumulative distance
    float vel_mps = (delta * K_DIST) / dt;      // instantaneous velocity


    // ============================
    //  JSON OUTPUT FOR ROS2 NODE
    // ============================
    Serial.print("{\"pulsos\":");
    Serial.print(current);
    Serial.print(",\"dist\":");
    Serial.print(dist_m, 4);
    Serial.print(",\"vel\":");
    Serial.print(vel_mps, 4);
    Serial.println("}");

    lastTime = now;
  }
}
