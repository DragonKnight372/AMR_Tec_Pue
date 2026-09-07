/*
=========================================================
 RP2040 Feather — SENSORES  (oficial)
 Encoder + BNO055 Adafruit
 JSON output para ROS2
=========================================================
*/

// =====================================================
// LIBRERIAS
// =====================================================
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// =====================================================
// BNO055
// =====================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// =====================================================
// ENCODER PINS
// =====================================================
#define ENCODER_A 24
#define ENCODER_B TX
#define ENCODER_Z RX

// =====================================================
// LED
// =====================================================
#define LED_PIN LED_BUILTIN

// =====================================================
// ENCODER VARIABLES
// =====================================================
volatile long contador = 0;
volatile int lastA = 0;

long lastCount = 0;

// =====================================================
// WHEEL + ENCODER CONFIG
// =====================================================

// diámetro rueda (metros)
float wheel_diameter = 0.56;

// pulsos por revolución
int pulses_per_rev = 10200;

float wheel_circumference = 0;
float K_DIST = 0;

// =====================================================
// TIMING
// =====================================================
unsigned long lastTime = 0;

// =====================================================
// ENCODER ISR
// =====================================================
void encoderISR() {

  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  if (A != lastA) {

    if (A == B)
      contador++;
    else
      contador--;
  }

  lastA = A;
}

// =====================================================
// INDEX ISR
// =====================================================
void indexISR() {

  // opcional:
  // contador = 0;
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  pinMode(LED_PIN, OUTPUT);

  // --------------------------
  // SERIAL
  // --------------------------
  Serial.begin(115200);

  // --------------------------
  // I2C  ← Wire usa SDA/SCL del Feather por defecto
  // --------------------------
  Wire.begin();

  // --------------------------
  // BNO055
  // --------------------------
  if (!bno.begin()) {

    Serial.println("No se detecta BNO055");

    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  // --------------------------
  // ENCODER
  // --------------------------
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    encoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_Z),
    indexISR,
    RISING
  );

  // --------------------------
  // K_DIST
  // --------------------------
  wheel_circumference = PI * wheel_diameter;

  K_DIST = wheel_circumference / pulses_per_rev;

  lastCount = contador;
  lastTime = millis();

  Serial.println("{\"status\":\"RP2040 Online\"}");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  unsigned long now = millis();

  if (now - lastTime >= 50) {

    // --------------------------
    // LED heartbeat
    // --------------------------
    static bool ledState = false;

    ledState = !ledState;

    digitalWrite(LED_PIN, ledState);

    // --------------------------
    // ENCODER
    // --------------------------
    long current = contador;

    long delta = current - lastCount;

    lastCount = current;

    float dt = (now - lastTime) / 1000.0;

    float dist_m = current * K_DIST;

    float vel_mps = (delta * K_DIST) / dt;

    // --------------------------
    // IMU
    // --------------------------
    imu::Vector<3> euler =
      bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    imu::Vector<3> gyro =
      bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    imu::Vector<3> accel =
      bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // --------------------------
    // JSON OUTPUT
    // --------------------------
    Serial.print("{");

    Serial.print("\"pulsos\":");
    Serial.print(current);

    Serial.print(",\"dist\":");
    Serial.print(dist_m, 4);

    Serial.print(",\"vel\":");
    Serial.print(vel_mps, 4);

    Serial.print(",\"ax\":");
    Serial.print(accel.x(), 3);

    Serial.print(",\"ay\":");
    Serial.print(accel.y(), 3);

    Serial.print(",\"az\":");
    Serial.print(accel.z(), 3);

    Serial.print(",\"gx\":");
    Serial.print(gyro.x(), 4);

    Serial.print(",\"gy\":");
    Serial.print(gyro.y(), 4);

    Serial.print(",\"gz\":");
    Serial.print(gyro.z(), 4);

    Serial.print(",\"yaw\":");
    Serial.print(euler.x(), 2);

    Serial.print(",\"pitch\":");
    Serial.print(euler.y(), 2);

    Serial.print(",\"roll\":");
    Serial.print(euler.z(), 2);

    Serial.print(",\"timestamp\":");
    Serial.print(now);

    Serial.println("}");

    lastTime = now;
  }
}