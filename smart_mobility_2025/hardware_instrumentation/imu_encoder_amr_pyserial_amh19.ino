/*=========================================================
   AMH19 - ESP32 Integrated Encoder + BNO055 IMU
   ------------------------------------------------
  Autor: Abraham Moro-Hernandez (AMH19)
  =========================================================
   Description:
   Reads an incremental encoder (channels A, B, Z) and a BNO055 IMU. 
   Combines all measurements into a single JSON packet at 20 Hz, 
   intended for ROS2 processing.

   Published data:
     pulsos      Raw encoder pulse count
     dist        Distance traveled in meters
     vel         Linear velocity in m/s
     ax, ay, az  Linear acceleration (m/s²)
     gx, gy, gz  Angular velocity (rad/s)
     yaw, pitch, roll  Euler orientation in degrees
     timestamp   Milliseconds since boot

   Notes:
     The value K_DIST is calibrated for the AMR1 Motor encoder producing 
     10200 pulses per wheel rotation.
*/


#include "BNO055_support.h"
#include <Wire.h>


// ==================================================
// Encoder Pins
// ==================================================
#define ENCODER_A 34
#define ENCODER_B 35
#define ENCODER_Z 32


// ==================================================
// I2C Configuration for BNO055
// ==================================================
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000


// ==================================================
// Status LED
// ==================================================
#define LED_PIN 2


// ==================================================
// Encoder Variables
// ==================================================
volatile long contador = 0;
volatile int lastA = 0;
float K_DIST = 0.000174;   // Wheel distance per encoder pulse
long lastCount = 0;


// ==================================================
// BNO055 Data Structures
// ==================================================
struct bno055_t myBNO;
struct bno055_euler myEulerData;
struct bno055_linear_accel myAccelData;
struct bno055_gyro myGyroData;


// ==================================================
// Timing (20 Hz)
// ==================================================
unsigned long lastTime = 0;


// ==================================================
// Encoder ISR - Channel A
// ==================================================
void IRAM_ATTR encoderA_ISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  if (A != lastA) {
    if (A == B) contador++;
    else        contador--;
  }

  lastA = A;
}


// ==================================================
// Encoder ISR - Channel Z
// ==================================================
void IRAM_ATTR encoderZ_ISR() {
  // Optional index-based reset or latch
}



void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Encoder pin setup
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), encoderZ_ISR, RISING);

  // I2C initialization
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // Serial output
  Serial.begin(115200);
  delay(1000);

  // BNO055 initialization
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  // Visual initialization indicator
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  Serial.println("{\"status\":\"ESP32 Encoder + BNO055 Online\"}");

  lastCount = contador;
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= 50) {

    // LED heartbeat
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);

    // Encoder reading
    long current = contador;
    long delta = current - lastCount;
    lastCount = current;

    float dt = (now - lastTime) / 1000.0;
    float dist_m = current * K_DIST;
    float vel_mps = (delta * K_DIST) / dt;

    // BNO055 readings
    bno055_read_euler_hrp(&myEulerData);
    bno055_read_linear_accel_xyz(&myAccelData);
    bno055_read_gyro_xyz(&myGyroData);

    // Combined JSON output
    Serial.print("{");

    Serial.print("\"pulsos\":");
    Serial.print(current);

    Serial.print(",\"dist\":");
    Serial.print(dist_m, 4);

    Serial.print(",\"vel\":");
    Serial.print(vel_mps, 4);

    Serial.print(",\"ax\":");
    Serial.print(float(myAccelData.x) / 100.0, 3);

    Serial.print(",\"ay\":");
    Serial.print(float(myAccelData.y) / 100.0, 3);

    Serial.print(",\"az\":");
    Serial.print(float(myAccelData.z) / 100.0, 3);

    Serial.print(",\"gx\":");
    Serial.print(float(myGyroData.x) / 900.0, 4);

    Serial.print(",\"gy\":");
    Serial.print(float(myGyroData.y) / 900.0, 4);

    Serial.print(",\"gz\":");
    Serial.print(float(myGyroData.z) / 900.0, 4);

    Serial.print(",\"yaw\":");
    Serial.print(float(myEulerData.h) / 16.0, 2);

    Serial.print(",\"pitch\":");
    Serial.print(float(myEulerData.p) / 16.0, 2);

    Serial.print(",\"roll\":");
    Serial.print(float(myEulerData.r) / 16.0, 2);

    Serial.print(",\"timestamp\":");
    Serial.print(now);

    Serial.println("}");

    lastTime = now;
  }
}


/*
   Hardware Connections

   Encoder:
     Channel A on GPIO 34
     Channel B on GPIO 35
     Channel Z on GPIO 32

   BNO055:
     VDD to 3.3V
     GND to GND
     SDA to GPIO 21
     SCL to GPIO 22
     PS0 = GND
     PS1 = GND

   Expected JSON output:
     {
       "pulsos": 1234,
       "dist": 0.2148,
       "vel": 0.5234,
       "ax": 0.123,
       "ay": -0.045,
       "az": 9.810,
       "gx": 0.0012,
       "gy": -0.0034,
       "gz": 0.0056,
       "yaw": 45.23,
       "pitch": 1.45,
       "roll": -0.89,
       "timestamp": 12345
     }
*/
