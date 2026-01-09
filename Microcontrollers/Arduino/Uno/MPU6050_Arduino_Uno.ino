/*
  MPU6050 3D IMU Visualization for Arduino Uno
  Author: Naman Harshwal
  License: MIT

  Complete firmware for reading MPU6050 sensor data and transmitting to Python visualizer
  via serial communication (115200 baud).

  Hardware: Arduino Uno + MPU6050 on I2C (A4=SDA, A5=SCL)
  Libraries Required: MPU6050, I2Cdev, Wire
*/

#include "Wire.h"
#include "MPU6050.h"
#include <math.h>

// MPU6050 object
MPU6050 mpu;

// IMU sensor variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temperature;

// Calibration offsets
int16_t accel_calib_x = 0, accel_calib_y = 0, accel_calib_z = 0;
int16_t gyro_calib_x = 0, gyro_calib_y = 0, gyro_calib_z = 0;

// Complementary filter variables
float roll = 0, pitch = 0, yaw = 0;
float ax_norm, ay_norm, az_norm;
float gx_rad, gy_rad, gz_rad;
float alpha = 0.98;  // Complementary filter constant
float dt = 0.01;     // Time step (100 Hz)
unsigned long lastTime = 0;

// Buffer for serial output
char jsonBuffer[128];

// Setup function - Initialize I2C, Serial, and MPU6050
void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n[INFO] Starting MPU6050 Arduino Uno Initialization...");
  
  // Initialize I2C communication
  Wire.begin();
  delay(100);
  
  // Initialize MPU6050
  mpu.initialize();
  delay(100);
  
  // Verify MPU6050 connection
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 connection failed!");
    Serial.println("[ERROR] Check wiring: A4=SDA, A5=SCL");
    while (1) {
      Serial.println(".");
      delay(1000);
    }
  }
  
  Serial.println("[SUCCESS] MPU6050 detected successfully");
  
  // Set full scale ranges
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);  // ±16g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250°/s
  
  // Enable digital low pass filter (21 Hz cutoff)
  // mpu.setDLPFMode(MPU6050_DLPF_BW_21);  // Disabled: constant not available in library  
  // Set sample rate divider (100 Hz sampling: 8000/(1+79))
  mpu.setRate(79);
  
  // Calibrate sensors
  calibrateSensors();
  
  lastTime = millis();
  Serial.println("[INFO] MPU6050 initialization complete");
  Serial.println("[INFO] Sending JSON data at 100 Hz...");
  delay(500);
}

// Main loop - Read sensors and transmit data
void loop() {
  // Calculate time delta
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Ensure minimum dt
  if (dt > 0.1) dt = 0.01;
  
  // Read accelerometer and gyroscope
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, gy, &gz);
  temperature = mpu.getTemperature();  // Get temperature in 16-bit 2's complement format  // Apply calibration offsets
  ax -= accel_calib_x;
  ay -= accel_calib_y;
  az -= accel_calib_z;
  gx -= gyro_calib_x;
  gy -= gyro_calib_y;
  gz -= gyro_calib_z;
  
  // Normalize accelerometer to g units
  ax_norm = ax / 2048.0;  // 16g range = 2048 LSB/g
  ay_norm = ay / 2048.0;
  az_norm = az / 2048.0;
  
  // Convert gyroscope to radians/second
  gx_rad = gx / 131.0;    // 250°/s range = 131 LSB/(°/s)
  gy_rad = gy / 131.0;
  gz_rad = gz / 131.0;
  
  // Convert to radians
  gx_rad = gx_rad * (3.14159 / 180.0);
  gy_rad = gy_rad * (3.14159 / 180.0);
  gz_rad = gz_rad * (3.14159 / 180.0);
  
  // Calculate pitch and roll from accelerometer
  float accel_roll = atan2(ay_norm, az_norm) * 180.0 / 3.14159;
  float accel_pitch = atan2(-ax_norm, sqrt(ay_norm*ay_norm + az_norm*az_norm)) * 180.0 / 3.14159;
  
  // Complementary filter: Combine accelerometer and gyroscope data
  roll = alpha * (roll + gx_rad * 180.0 / 3.14159 * dt) + (1 - alpha) * accel_roll;
  pitch = alpha * (pitch + gy_rad * 180.0 / 3.14159 * dt) + (1 - alpha) * accel_pitch;
  yaw = yaw + gz_rad * 180.0 / 3.14159 * dt;
  
  // Keep yaw within -180 to 180 degrees
  while (yaw > 180) yaw -= 360;
  while (yaw < -180) yaw += 360;
  
  // Prepare JSON data with proper framing
  snprintf(jsonBuffer, sizeof(jsonBuffer), 
           "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"temp\":%.1f}\n",
           roll, pitch, yaw, ax_norm, ay_norm, az_norm, temperature / 340.0 + 36.53);
  
  // Transmit JSON data
  Serial.write((uint8_t*)jsonBuffer, strlen(jsonBuffer));
  
  // Maintain 100 Hz sampling rate
  delay(10);
}

// Calibrate accelerometer and gyroscope
void calibrateSensors() {
  Serial.println("[INFO] Calibrating sensors (keep device still)...");
  
  int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
  int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  int samples = 200;
  
  for (int i = 0; i < samples; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    
    accel_x_sum += ax;
    accel_y_sum += ay;
    accel_z_sum += az;
    gyro_x_sum += gx;
    gyro_y_sum += gy;
    gyro_z_sum += gz;
    
    delay(10);
    
    if (i % 50 == 0) Serial.print(".");
  }
  
  accel_calib_x = accel_x_sum / samples;
  accel_calib_y = accel_y_sum / samples;
  accel_calib_z = (accel_z_sum / samples) - 2048;  // Subtract 1g for Z-axis
  gyro_calib_x = gyro_x_sum / samples;
  gyro_calib_y = gyro_y_sum / samples;
  gyro_calib_z = gyro_z_sum / samples;
  
  Serial.println("\n[SUCCESS] Calibration complete");
  Serial.print("[INFO] Accel offsets: ");
  Serial.print(accel_calib_x); Serial.print(", ");
  Serial.print(accel_calib_y); Serial.print(", ");
  Serial.println(accel_calib_z);
}
