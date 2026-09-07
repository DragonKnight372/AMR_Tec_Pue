// ======================================================================
//  AMH19-IVDT ESP32 BNO055 IMU Interface
//  ---------------------------------------------------------------------
//  Author: Abraham Moro-Hernandez (AMH19)
//  Co-Author: Ivan Valdez del Toro
//
//  Description:
//      Reads Euler orientation, linear acceleration, and gyroscope data
//      from the BNO055 sensor using Bosch's BNO055_support driver.
//      Publishes compact JSON frames suitable for ROS2 nodes that
//      perform pose estimation.
//
//  Output Rate:
//      20 Hz (50 ms)
//
//  Units and scaling:
//      Linear acceleration: raw / 100.0  in m/s²
//      Angular velocity:    raw / 900.0  in rad/s
//      Euler angles:        raw / 16.0   in degrees
//
//  Hardware notes:
//      ESP32 I2C with SDA = 21 and SCL = 22
//      BNO055 must be in I2C mode (PS0 = 0, PS1 = 0)
// ======================================================================


#include "BNO055_support.h"
#include <Wire.h>


// I2C configuration for ESP32
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000


// Status LED on ESP32
#define LED_PIN 2


// BNO055 data structures
struct bno055_t myBNO;
struct bno055_euler myEulerData;
struct bno055_linear_accel myAccelData;
struct bno055_gyro myGyroData;


// Timing control for 20 Hz publishing
unsigned long lastTime = 0;


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // Serial port for JSON output
  Serial.begin(115200);
  delay(1000);

  // Initialize BNO055 IMU
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  // Boot indication
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  Serial.println("{\"status\":\"BNO055 initialized\"}");
}


void loop()
{
  if ((millis() - lastTime) >= 50)
  {
    lastTime = millis();

    // Heartbeat LED for activity monitoring
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);

    // Read IMU data
    bno055_read_euler_hrp(&myEulerData);
    bno055_read_linear_accel_xyz(&myAccelData);
    bno055_read_gyro_xyz(&myGyroData);

    // JSON formatted sensor output
    Serial.print("{");

    Serial.print("\"ax\":");
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
    Serial.print(lastTime);

    Serial.println("}");
  }
}


/*
  
  Wiring:
    BNO055 VDD to 3.3V
    BNO055 GND to GND
    BNO055 SDA to GPIO 21
    BNO055 SCL to GPIO 22
    BNO055 PS0 = GND
    BNO055 PS1 = GND
*/
