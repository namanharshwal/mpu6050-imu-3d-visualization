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

// Calibration offsets
int16_t accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
int16_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

// Current orientation angles
float roll = 0, pitch = 0, yaw = 0;

// Complementary filter coefficients
const float alpha = 0.95;  // Gyroscope weight
const float beta = 0.05;   // Accelerometer weight

// Timing variables
unsigned long lastTime = 0;
float dt = 0.01;  // Time step (10ms for 100Hz)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(100);
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Initialize MPU6050
  Serial.println("\n=== MPU6050 Arduino Uno Initialization ===");
  mpu.initialize();
  
  // Check if MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 connection failed!");
    while(1);
  }
  
  Serial.println("MPU6050 Connected Successfully");
  Serial.println("WHO_AM_I: 0x" + String(mpu.getDeviceID(), HEX));
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250 deg/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);              // 5Hz bandwidth
  
  Serial.println("Gyro Range: ±250 deg/s");
  Serial.println("Accel Range: ±2g");
  Serial.println("Sample Rate: 100 Hz");
  
  // Calibrate sensors
  Serial.println("\nCalibrating... Keep device still!");
  calibrateSensors();
  
  Serial.println("\nCalibration Complete!");
  Serial.println("Starting data transmission...");
  Serial.println("=== Sensor Data Streaming ===");
  Serial.println("{JSON format at 10ms intervals}");
  
  lastTime = millis();
}

void loop() {
  // Calculate time delta
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Constrain dt to prevent large jumps
  if (dt > 0.1) dt = 0.01;
  if (dt < 0.005) dt = 0.01;
  
  // Read sensor data
  int16_t ax, ay, az, gx, gy, gz, temp;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = mpu.getTemperature();
  
  // Convert raw values to physical units
  float accel_x = (ax / 16384.0) * 9.81 - (accel_offset_x / 16384.0);
  float accel_y = (ay / 16384.0) * 9.81 - (accel_offset_y / 16384.0);
  float accel_z = (az / 16384.0) * 9.81 - (accel_offset_z / 16384.0);
  
  float gyro_x = (gx / 131.0) - (gyro_offset_x / 131.0);
  float gyro_y = (gy / 131.0) - (gyro_offset_y / 131.0);
  float gyro_z = (gz / 131.0) - (gyro_offset_z / 131.0);
  
  float temp_c = (temp / 340.0) + 36.53;
  
  // Apply complementary filter
  complementaryFilter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt);
  
  // Send JSON data
  sendJSONData(currentTime, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_c);
  
  // Maintain 100Hz sampling
  delay(10);
}

void calibrateSensors() {
  int samples = 100;
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    
    delay(10);
  }
  
  accel_offset_x = sum_ax / samples;
  accel_offset_y = sum_ay / samples;
  accel_offset_z = (sum_az / samples) - 16384;  // Remove 1g from Z
  
  gyro_offset_x = sum_gx / samples;
  gyro_offset_y = sum_gy / samples;
  gyro_offset_z = sum_gz / samples;
  
  Serial.print("Accel Offset: ");
  Serial.print(accel_offset_x); Serial.print(", ");
  Serial.print(accel_offset_y); Serial.print(", ");
  Serial.println(accel_offset_z);
  
  Serial.print("Gyro Offset: ");
  Serial.print(gyro_offset_x); Serial.print(", ");
  Serial.print(gyro_offset_y); Serial.print(", ");
  Serial.println(gyro_offset_z);
}

void complementaryFilter(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  // Calculate roll and pitch from accelerometer
  float accel_roll = atan2(ay, az) * 180.0 / M_PI;
  float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
  
  // Apply complementary filter
  roll = alpha * (roll + gx * dt) + beta * accel_roll;
  pitch = alpha * (pitch + gy * dt) + beta * accel_pitch;
  yaw += gz * dt;  // Yaw from gyroscope only
}

void sendJSONData(unsigned long timestamp, float ax, float ay, float az, float gx, float gy, float gz, float temp) {
  Serial.print("{\"timestamp\":");
  Serial.print(timestamp);
  Serial.print(",\"accel_x\":");
  Serial.print(ax, 4);
  Serial.print(",\"accel_y\":");
  Serial.print(ay, 4);
  Serial.print(",\"accel_z\":");
  Serial.print(az, 4);
  Serial.print(",\"gyro_x\":");
  Serial.print(gx, 4);
  Serial.print(",\"gyro_y\":");
  Serial.print(gy, 4);
  Serial.print(",\"gyro_z\":");
  Serial.print(gz, 4);
  Serial.print(",\"roll\":");
  Serial.print(roll, 2);
  Serial.print(",\"pitch\":");
  Serial.print(pitch, 2);
  Serial.print(",\"yaw\":");
  Serial.print(yaw, 2);
  Serial.print(",\"temp\":");
  Serial.print(temp, 2);
  Serial.println("}");
}
