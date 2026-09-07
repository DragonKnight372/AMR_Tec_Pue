// ======================================================================
//  IVDT-AMH19 - ESP32 BNO055 Orientation Reader 
// ----------------------------------------------------------------------
//  Author: Ivan Valdez del Toro
//  Co-Author: Abraham Moro-Hernandez (AMH19)
//
//  Description:
//      Reads orientation (Heading/Yaw, Roll, Pitch) from the BNO055
//      sensor using the official Bosch "BNO055_support.h" driver.  
//      Outputs values every 100 ms over serial for debugging,
//      calibration, or integration with external systems.
//
//  Sensor Mode:
//      OPERATION_MODE_NDOF  full 9-DOF fusion (acc + gyro + mag)
//      Provides stable absolute orientation in degrees.
//
//  Notes:
//      - BNO055 Euler registers output values scaled by 1/16
//        (raw_value / 16 = degrees)
//      - Using custom I2C pins (ESP32 default SDA=21, SCL=22 allowed)
// ======================================================================


#include "BNO055_support.h"
#include <Wire.h>


// =========================
//  I2C PIN CONFIG (ESP32)
// =========================
#define SDA_PIN 21
#define SCL_PIN 22


// =========================
//  BNO055 DATA STRUCTURES
// =========================
// 'myBNO' stores device information
// 'myEulerData' stores Euler angles (h, r, p) in raw 1/16 degree format
struct bno055_t myBNO;
struct bno055_euler myEulerData;


// =========================
//  TIMING CONTROL (100 ms)
// =========================
unsigned long lastTime = 0;



void setup()
{
  // Initialize I2C (explicit pins for ESP32)
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize BNO055 and set fusion mode
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  // Serial output for monitoring
  Serial.begin(115200);
}



void loop()
{
  if ((millis() - lastTime) >= 100)
  {
    lastTime = millis();

    // Read Euler orientation data (Heading, Roll, Pitch)
    bno055_read_euler_hrp(&myEulerData);

    // -------------------------
    //  PRINT ORIENTATION DATA
    // -------------------------
    Serial.print("Time Stamp: ");
    Serial.println(lastTime);

    Serial.print("Heading (Yaw): ");
    Serial.println(float(myEulerData.h) / 16.00);

    Serial.print("Roll: ");
    Serial.println(float(myEulerData.r) / 16.00);

    Serial.print("Pitch: ");
    Serial.println(float(myEulerData.p) / 16.00);

    Serial.println();
  }
}
