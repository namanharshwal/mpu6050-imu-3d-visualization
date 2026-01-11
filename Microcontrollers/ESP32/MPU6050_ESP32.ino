/*
 * MPU6050 ESP32 3D Visualization Firmware
 * Author: Naman Harshwal
 * Date: January 2026
 * Version: 1.0.0
 * 
 * This firmware reads MPU6050 sensor data on ESP32 and transmits
 * real-time orientation (roll, pitch, yaw) via serial communication
 * for 3D visualization with Python scripts.
 */

#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>
#include <math.h>

// MPU6050 I2C Address
#define MPU_ADDR 0x68

// Sampling configuration
#define SAMPLE_RATE 100  // Hz
#define BAUD_RATE 115200  // ESP32 supports higher baud rates

// Sensor sensitivity constants
const float GYRO_SENSITIVITY = 131.0;      // LSB/°/s for ±250°/s range
const float ACCEL_SENSITIVITY = 16384.0;   // LSB/g for ±2g range

// Complementary filter coefficient
const float ALPHA = 0.98;  // Weight for gyroscope (higher = more gyro trust)
const float DT = 1.0 / SAMPLE_RATE;  // Time interval between samples

// Global variables
MPU6050 mpu;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll = 0, pitch = 0, yaw = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

unsigned long lastTime = 0;
int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);
  
  Serial.println("\n\nMPU6050 ESP32 Initialization Starting...");
  
  // Initialize I2C
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22 for ESP32
  Wire.setClock(400000);  // 400kHz I2C clock
  
  // Initialize MPU6050
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 connection failed!");
    while (1) {
      Serial.println("Retrying connection...");
      delay(1000);
      mpu.initialize();
      if (mpu.testConnection()) break;
    }
  }
  
  Serial.println("✓ MPU6050 connected successfully");
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
  mpu.setDLPFMode(MPU6050_DLPF_BW_184);  // Digital low-pass filter
  mpu.setSleepEnabled(false);
  
  Serial.println("✓ MPU6050 configured");
  
  // Perform calibration
  calibrateSensor();
  
  lastTime = millis();
  Serial.println("✓ Initialization complete. Streaming data...");
  Serial.println("{\"status\": \"ready\"}");
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  
  // Maintain consistent sampling rate
  if (elapsedTime < (1000 / SAMPLE_RATE)) {
    return;  // Wait until next sample time
  }
  
  lastTime = currentTime;
  float dt = elapsedTime / 1000.0;  // Convert to seconds
  
  // Read raw sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert raw data to physical units
  accelX = (ax / ACCEL_SENSITIVITY);
  accelY = (ay / ACCEL_SENSITIVITY);
  accelZ = (az / ACCEL_SENSITIVITY);
  
  gyroX = (gx / GYRO_SENSITIVITY) - gyroXoffset;
  gyroY = (gy / GYRO_SENSITIVITY) - gyroYoffset;
  gyroZ = (gz / GYRO_SENSITIVITY) - gyroZoffset;
  
  // Calculate angles from accelerometer (more stable but noisy)
  float accelRoll = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = asin(-accelX / 9.81) * 180.0 / PI;
  
  // Complementary filter: combine gyro (fast, drift) and accel (stable, noisy)
  roll = ALPHA * (roll + gyroX * dt) + (1 - ALPHA) * accelRoll;
  pitch = ALPHA * (pitch + gyroY * dt) + (1 - ALPHA) * accelPitch;
  yaw = yaw + gyroZ * dt;
  
  // Constrain angles to -180 to 180 degrees
  roll = constrainAngle(roll);
  pitch = constrainAngle(pitch);
  yaw = constrainAngle(yaw);
  
  // Send data as JSON
  sendJSON();
}

void calibrateSensor() {
  Serial.println("\n=== Starting Calibration ===");
  Serial.println("Keep MPU6050 stationary on flat surface...");
  
  int samples = 500;  // Number of samples for calibration
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  float targetAZ = 9.81;  // 1g in Z direction when flat
  
  // Blink LED pattern to indicate calibration
  for (int i = 0; i < samples; i++) {
    if (i % 50 == 0) {
      Serial.print(".");
    }
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    sumGX += gx / GYRO_SENSITIVITY;
    sumGY += gy / GYRO_SENSITIVITY;
    sumGZ += gz / GYRO_SENSITIVITY;
    
    sumAX += ax / ACCEL_SENSITIVITY;
    sumAY += ay / ACCEL_SENSITIVITY;
    sumAZ += az / ACCEL_SENSITIVITY;
    
    delay(10);
  }
  
  // Calculate offsets
  gyroXoffset = sumGX / samples;
  gyroYoffset = sumGY / samples;
  gyroZoffset = sumGZ / samples;
  
  accelXoffset = sumAX / samples;
  accelYoffset = sumAY / samples;
  accelZoffset = (sumAZ / samples) - targetAZ;
  
  Serial.println("\n✓ Calibration complete!");
  Serial.print("Gyro offsets: X=");
  Serial.print(gyroXoffset);
  Serial.print(" Y=");
  Serial.print(gyroYoffset);
  Serial.print(" Z=");
  Serial.println(gyroZoffset);
}

float constrainAngle(float angle) {
  if (angle > 180) {
    angle -= 360;
  }
  if (angle < -180) {
    angle += 360;
  }
  return angle;
}

void sendJSON() {
  // Create JSON document
  StaticJsonDocument<200> doc;
  
  doc["roll"] = serialized(String(roll, 2));
  doc["pitch"] = serialized(String(pitch, 2));
  doc["yaw"] = serialized(String(yaw, 2));
  doc["temp"] = serialized(String(getTemperature(), 1));
  
  // Serialize and send
  serializeJson(doc, Serial);
  Serial.println();
}

float getTemperature() {
  // MPU6050 internal temperature sensor
  // Temp (°C) = (TEMP_OUT / 340) + 36.53
  int16_t tempRaw = mpu.getTemperature();
  return (tempRaw / 340.0) + 36.53;
}
