/*
  MPU6050 3D IMU Visualization for Arduino Uno
  Author: Naman Harshwal
  License: MIT
  
  Complete firmware for reading MPU6050 sensor data and transmitting to Python visualizer
  via serial communication (115200 baud).
  
  Hardware: Arduino Uno + MPU6050 on I2C (A4=SDA, A5=SCL)
  Libraries Required: MPU6050, I2Cdev, Wire
  
  FIXES APPLIED:
  - Correct Euler angle calculations from gravity vector
  - Consistent unit conversions (degrees throughout)
  - Proper complementary filter with degree-based integration
  - Correct gyroscope sensitivity (131 LSB/deg/s for +/-250 deg/s)
  - Correct accel sensitivity (16384 LSB/g for +/-2g)
  - Robust JSON output with NaN/Inf checking
  - Yaw wrapping to ±180 degrees
*/

#include "Wire.h"
#include "MPU6050.h"
#include <math.h>

// MPU6050 object
MPU6050 mpu;

// IMU sensor variables (raw readings)
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temperature;

// Calibration offsets
int16_t accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
int16_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

// MPU6050 sensitivity constants
const float ACCEL_SENSITIVITY = 16384.0;  // LSB/g for +/-2g range (default)
const float GYRO_SENSITIVITY = 131.0;     // LSB/(deg/s) for +/-250 deg/s range (default)

// Orientation angles (in degrees)
float roll = 0.0, pitch = 0.0, yaw = 0.0;

// Complementary filter constants
const float ALPHA = 0.98;              // Gyro weight (0.98 = 98% gyro, 2% accel)
const float SAMPLE_RATE = 100.0;       // Hz (10ms loop)
const float DT = 1.0 / SAMPLE_RATE;    // Time step in seconds (0.01s)

// Timing
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Yaw mode control
bool yaw_enabled = true;

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  
  // Test connection
  if (!mpu.testConnection()) {
    Serial.println("{\"error\":\"MPU6050 connection failed\"}");
    while (1) {
      delay(1000);
    }
  }
  
  delay(100);
  
  // Start calibration
  Serial.println("{\"status\":\"Calibrating - keep device still for 6 seconds\"}");
  delay(500);
  calibrateSensors();
  
  Serial.println("{\"status\":\"Calibration complete, starting visualization\"}");
  delay(500);
  
  lastTime = millis();
}

void loop() {
  // Calculate time delta
  currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Limit dt to prevent instability (cap at 100ms if loop is too slow)
  if (dt > 0.1) dt = 0.01;
  
  // Read raw sensor data
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  // Apply calibration offsets
  ax -= accel_offset_x;
  ay -= accel_offset_y;
  az -= accel_offset_z;
  gx -= gyro_offset_x;
  gy -= gyro_offset_y;
  gz -= gyro_offset_z;
  
  // Convert raw accelerometer values to g (gravity units)
  float ax_g = ax / ACCEL_SENSITIVITY;
  float ay_g = ay / ACCEL_SENSITIVITY;
  float az_g = az / ACCEL_SENSITIVITY;
  
  // Convert raw gyroscope values to degrees/second
  float gx_deg_s = gx / GYRO_SENSITIVITY;
  float gy_deg_s = gy / GYRO_SENSITIVITY;
  float gz_deg_s = gz / GYRO_SENSITIVITY;
  
  // Calculate roll and pitch from accelerometer
  // Using gravity vector to get absolute orientation
  // Roll: rotation around X-axis (left-right tilt)
  float accel_roll = atan2(ay_g, az_g) * (180.0 / M_PI);
  
  // Pitch: rotation around Y-axis (forward-backward tilt)
  float accel_pitch = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * (180.0 / M_PI);
  
  // Complementary filter for roll and pitch
  // Combine gyro (fast response) with accel (drift correction)
  roll = ALPHA * (roll + gx_deg_s * dt) + (1.0 - ALPHA) * accel_roll;
  pitch = ALPHA * (pitch + gy_deg_s * dt) + (1.0 - ALPHA) * accel_pitch;
  
  // Yaw integration (gyro only, accel cannot measure yaw)
  if (yaw_enabled) {
    yaw += gz_deg_s * dt;
    
    // Wrap yaw to ±180 degrees
    if (yaw > 180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;
  }
  
  // Send JSON data to Python visualizer
  sendJSONData();
  
  // Check for commands from Python
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z') {
      yaw_enabled = !yaw_enabled;
      if (yaw_enabled) {
        yaw = 0.0;  // Reset yaw when re-enabling
      }
      Serial.println("{\"info\":\"Yaw mode toggled\"}");
    }
  }
  
  // Maintain 100 Hz update rate (10ms delay)
  delay(10);
}

void calibrateSensors() {
  const int SAMPLES = 600;  // 6 seconds at 100 Hz
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    
    if (i % 100 == 0) {
      Serial.print(".");
    }
    delay(10);
  }
  
  // Calculate average offsets
  accel_offset_x = ax_sum / SAMPLES;
  accel_offset_y = ay_sum / SAMPLES;
  // Subtract 1g (16384 LSB) from Z-axis since gravity acts on Z
  accel_offset_z = (az_sum / SAMPLES) - (int16_t)ACCEL_SENSITIVITY;
  
  gyro_offset_x = gx_sum / SAMPLES;
  gyro_offset_y = gy_sum / SAMPLES;
  gyro_offset_z = gz_sum / SAMPLES;
  
  // Print calibration results
  Serial.println();
  Serial.print("{\"accel_x\":");
  Serial.print(accel_offset_x);
  Serial.print(",\"accel_y\":");
  Serial.print(accel_offset_y);
  Serial.print(",\"accel_z\":");
  Serial.print(accel_offset_z);
  Serial.print(",\"gyro_x\":");
  Serial.print(gyro_offset_x);
  Serial.print(",\"gyro_y\":");
  Serial.print(gyro_offset_y);
  Serial.print(",\"gyro_z\":");
  Serial.print(gyro_offset_z);
  Serial.println("}");
}

void sendJSONData() {
  // Send JSON format: {"roll":X.XX,"pitch":X.XX,"yaw":X.XX}
  // This matches the expected format in Python visualization script
  Serial.print("{\"roll\":");
  printFloatWithPrecision(roll, 2);
  Serial.print(",\"pitch\":");
  printFloatWithPrecision(pitch, 2);
  Serial.print(",\"yaw\":");
  printFloatWithPrecision(yaw, 2);
  Serial.println("}");
}

// Helper function to print float with fixed decimal places
// Prevents NaN and Inf values from corrupting JSON output
void printFloatWithPrecision(float value, int precision) {
  // Safety check: replace NaN or Inf with 0.0
  if (isnan(value) || isinf(value)) {
    Serial.print("0");
    return;
  }
  
  // Print with specified precision
  Serial.print(value, precision);
}
