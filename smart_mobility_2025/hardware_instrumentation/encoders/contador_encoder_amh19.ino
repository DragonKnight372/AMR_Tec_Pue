// ======================================================================
//  AMH19 - Basic Encoder Test (ESP32 DevKit V1 - 38 pins)
//  Author: Abraham Moro-Hernandez (AMH19)
// ----------------------------------------------------------------------
//  Purpose:
//      Initial practical test to understand quadrature encoder readings
//      on the ESP32 using interrupts. This script measures:
//
//        • Pulses accumulated (variable 'contador')
//        • Direction of rotation (based on A/B phase relation)
//        • Index Z rising event (absolute reference position)
//        • Estimated RPM (computed every 100 ms)
//
//  Notes:
//      - Encoder resolution used: 1024 PPR
//      - ISR routines run in IRAM (ESP32 requirement)
// ======================================================================


// =========================
//  PIN DEFINITIONS (ESP32)
// =========================
#define ENCODER_A 34      // Channel A
#define ENCODER_B 35      // Channel B
#define ENCODER_Z 32      // Index pulse (optional)

#define PPR 1024          // Pulses Per Revolution of the encoder


// ===================================
//  GLOBAL VARIABLES (INTERRUPT-SAFE)
// ===================================
volatile long contador = 0;   // accumulated pulse count
volatile int lastA = 0;       // last state of channel A (for direction decoding)
volatile long indexPos = 0;   // stores count when Z-index triggers


// ============================================
//  ISR: Channel A Change  (Quadrature Decoder)
// ============================================
//  Determines direction based on A/B relationship
void IRAM_ATTR encoderA_ISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  if (A != lastA) {
    if (A == B) contador++;   // forward direction
    else        contador--;   // reverse direction
  }

  lastA = A;
}


// ===============================
//  ISR: Index Z Rising Detection
// ===============================
//  Captures the exact position at the reference mark
void IRAM_ATTR encoderZ_ISR() {
  indexPos = contador;
}



void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), encoderZ_ISR, RISING);

  Serial.println("Encoder 1024PPR ready");
}


// ======================
//  LOOP: RPM Estimation
// ======================
//  Computes RPM every 100 ms using pulse difference
long lastCount = 0;
unsigned long lastTime = 0;

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= 100) {  

    long delta = contador - lastCount;  // pulses in 100 ms
    lastCount = contador;

    float vueltas = (float)delta / PPR;     // revolutions per 0.1 s
    float rpm = (vueltas / 0.1) * 60.0;     // convert to revolutions/minute

    Serial.print("Pulsos: ");
    Serial.print(contador);
    Serial.print(" | Index en: ");
    Serial.print(indexPos);
    Serial.print(" | RPM: ");
    Serial.println(rpm);

    lastTime = now;
  }
}
