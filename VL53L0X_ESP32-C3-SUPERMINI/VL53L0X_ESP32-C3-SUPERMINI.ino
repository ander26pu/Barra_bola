#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ESP32-C3 Supermini I2C pins
#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_VL53L0X lox;

// Kalman filter variables
float q = 0.125;   // process noise covariance
float r = 8.0;     // measurement noise covariance
float x_est = 0;   // estimated value
float p_est = 1;   // estimation error covariance

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // I2C init
  Wire.begin(I2C_SDA, I2C_SCL);

  // sensor init
  if (!lox.begin()) {
    Serial.println("VL53L0X init failed");
    while (1) delay(100);
  }

  // CSV header for Arduino plotter
  Serial.println("Raw,Filtered");
}

void loop() {
  // take measurement
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  float z = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0;

  // Kalman prediction
  float x_pred = x_est;
  float p_pred = p_est + q;

  // Kalman update
  float k_gain = p_pred / (p_pred + r);
  x_est = x_pred + k_gain * (z - x_pred);
  p_est = (1 - k_gain) * p_pred;

  // output CSV: raw,filtered
  Serial.printf("%.0f,%.2f\n", z, x_est);

  delay(50);  // adjust sampling rate if needed
}
