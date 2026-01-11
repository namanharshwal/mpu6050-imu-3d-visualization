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

// Yaw mode control
bool yaw_enabled = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("{\"error\": \"MPU6050 connection failed\"}");
    while (1);
  }
  
  // Calibration routine
  Serial.println("{\"status\": \"Starting calibration...\"}");
  calibrate();
  Serial.println("{\"status\": \"Calibration complete\"}");
  
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Read raw sensor data
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getTemperature(&temperature);
  
  // Apply calibration offsets
  ax -= accel_calib_x;
  ay -= accel_calib_y;
  az -= accel_calib_z;
  gx -= gyro_calib_x;
  gy -= gyro_calib_y;
  gz -= gyro_calib_z;
  
  // Normalize accelerometer values
  float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
  ax_norm = ax / accel_magnitude;
  ay_norm = ay / accel_magnitude;
  az_norm = az / accel_magnitude;
  
  // Convert gyro to radians per second (sensitivity: 131 LSB/dps)
  gx_rad = (gx / 131.0) * (3.14159 / 180.0);
  gy_rad = (gy / 131.0) * (3.14159 / 180.0);
  gz_rad = (gz / 131.0) * (3.14159 / 180.0);
  
  // Complementary filter
  // Calculate pitch and roll from accelerometer
  float accel_pitch = atan2(ax_norm, sqrt(ay_norm*ay_norm + az_norm*az_norm)) * (180.0 / 3.14159);
  float accel_roll = atan2(ay_norm, az_norm) * (180.0 / 3.14159);
  
  // Integrate gyro data
  pitch = alpha * (pitch + gy_rad * dt) + (1 - alpha) * accel_pitch;
  roll = alpha * (roll + gx_rad * dt) + (1 - alpha) * accel_roll;
  
  if (yaw_enabled) {
    yaw += gz_rad * dt;
  }
  
  // Send data as JSON
  sendJSONData();
  
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z') {
      yaw_enabled = !yaw_enabled;
      Serial.println("{\"info\": \"Yaw mode toggled\"}");
    }
  }
  
  delay(10);  // 100 Hz update rate
}

void calibrate() {
  int samples = 600;  // 6 seconds at 100 Hz
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    
    if (i % 50 == 0) Serial.print(".");
    delay(10);
  }
  
  accel_calib_x = ax_sum / samples;
  accel_calib_y = ay_sum / samples;
  accel_calib_z = (az_sum / samples) - 2048;  // Subtract 1g for Z-axis
  gyro_calib_x = gx_sum / samples;
  gyro_calib_y = gy_sum / samples;
  gyro_calib_z = gz_sum / samples;
  
  Serial.println();
  Serial.println("{\"[INFO] Accel offsets: \"}");
  Serial.print(accel_calib_x); Serial.print(", ");
  Serial.print(accel_calib_y); Serial.print(", ");
  Serial.println(accel_calib_z);
}

void sendJSONData() {
  // Send roll, pitch, yaw as JSON
  Serial.print("{\"roll\":");
  Serial.print(roll);
  Serial.print(",\"pitch\":");
  Serial.print(pitch);
  Serial.print(",\"yaw\":");
  Serial.print(yaw);
  Serial.println("}");
}
