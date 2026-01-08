/*
 * ESP8266 NodeMCU v3 - MPU6050 IMU Sensor Firmware
 * Real-time Orientation Sensing via Serial Communication
 * Optimized for memory-constrained ESP8266 devices
 * 
 * Author: Naman Harshwal
 * Date: January 2026
 * Version: 1.0.0
 * 
 * REQUIRES LIBRARIES:
 * - I2Cdev by Jeff Rowberg
 * - MPU6050 by Electronic Cats or Jeff Rowberg
 * 
 * PINOUT (ESP8266 NodeMCU v3 -> MPU6050):
 * D4 (GPIO2)  -> SDA (Serial Data)
 * D3 (GPIO4)  -> SCL (Serial Clock)
 * 3.3V        -> VCC
 * GND         -> GND
 * D8 (GPIO15) -> INT (Optional, for interrupts)
 */

#include "MPU6050.h"
#include <Wire.h>

// ============================================================================
// CONFIGURATION SECTION
// ============================================================================

// I2C Pin Configuration for ESP8266 NodeMCU v3
#define SDA_PIN D4  // GPIO2 - Serial Data Line
#define SCL_PIN D3  // GPIO4 - Serial Clock Line

// I2C Clock Speed (400kHz is standard for MPU6050)
#define I2C_SPEED 400000

// Serial Communication Baud Rate
#define SERIAL_BAUD 115200

// Update Rate in milliseconds (100ms = 10Hz)
#define UPDATE_RATE_MS 100

// MPU6050 I2C Address (0x68 = default, 0x69 = alternate with AD0 pulled high)
#define MPU6050_ADDR 0x68

// ============================================================================
// VARIABLE DECLARATIONS
// ============================================================================

MPU6050 mpu(MPU6050_ADDR);  // Create MPU6050 object with default address

// Raw sensor data
int16_t accelX, accelY, accelZ;  // Acceleration (3-axis)
int16_t gyroX, gyroY, gyroZ;      // Gyroscope (3-axis)
int16_t tempRaw;                   // Temperature (raw)

// Processed orientation angles (degrees)
float roll = 0.0;    // Rotation around X-axis
float pitch = 0.0;   // Rotation around Y-axis
float yaw = 0.0;     // Rotation around Z-axis (not calculated without magnetometer)

// Complementary filter coefficients
const float ALPHA = 0.98;  // Gyroscope weight (0.98 = 98% gyro, 2% accel)
const float G_CONVERT = 131.0;    // Gyroscope conversion factor (for 250 deg/s range)
const float A_CONVERT = 16384.0;  // Accelerometer conversion factor (for 2g range)
const float dt = UPDATE_RATE_MS / 1000.0;  // Time delta in seconds

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long currentTime = 0;

// Status flags
bool sensorInitialized = false;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
    // Initialize Serial Communication
    Serial.begin(SERIAL_BAUD);
    delay(1000);  // Wait for serial monitor to connect
    
    // Clear any previous output
    Serial.println();
    Serial.println();
    Serial.println("========================================");
    Serial.println("  ESP8266 MPU6050 IMU Initialization");
    Serial.println("========================================");
    
    // Initialize I2C Communication
    Serial.println("[INFO] Initializing I2C communication...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_SPEED);
    delay(100);
    
    // Initialize MPU6050
    Serial.println("[INFO] Initializing MPU6050 sensor...");
    mpu.initialize();
    
    // Test MPU6050 Connection
    Serial.println("[INFO] Testing MPU6050 connection...");
    if (!mpu.testConnection()) {
        Serial.println("[ERROR] MPU6050 connection FAILED!");
        Serial.println("[ERROR] Check wiring:");
        Serial.println("        NodeMCU D4 (GPIO2) -> MPU6050 SDA");
        Serial.println("        NodeMCU D3 (GPIO4) -> MPU6050 SCL");
        Serial.println("        NodeMCU 3.3V       -> MPU6050 VCC");
        Serial.println("        NodeMCU GND        -> MPU6050 GND");
        Serial.println("[INFO] Entering infinite loop. Check connections and restart.");
        while (1) {  // Halt execution
            delay(1000);
            Serial.println(".");
        }
    }
    Serial.println("[SUCCESS] MPU6050 connected successfully!");
    
    // Configure MPU6050 (Optional - using default settings)
    Serial.println("[INFO] Configuring MPU6050...");
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // Set accelerometer range to 2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // Set gyroscope range to 250 deg/s
    mpu.setSleepEnabled(false);  // Disable sleep mode
    delay(100);
    
    Serial.println("[SUCCESS] MPU6050 configured!");
    Serial.println();
    Serial.println("Starting sensor data stream...");
    Serial.println("Format: {\"roll\": X.XX, \"pitch\": Y.YY, \"yaw\": Z.ZZ}");
    Serial.println();
    Serial.println("========================================");
    
    sensorInitialized = true;
    lastUpdateTime = millis();
}

// ============================================================================
// LOOP FUNCTION
// ============================================================================

void loop() {
    if (!sensorInitialized) return;  // Exit if sensor not initialized
    
    currentTime = millis();
    
    // Check if it's time to update (UPDATE_RATE_MS interval)
    if (currentTime - lastUpdateTime >= UPDATE_RATE_MS) {
        lastUpdateTime = currentTime;
        
        // Read raw sensor data from MPU6050
        mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
        
        // Calculate orientation angles using accelerometer
        // These are approximate angles calculated from accelerometer data only
        // More accurate if combined with gyroscope data (complementary filter)
        
        // Accelerometer-based angle calculation
        float accelRoll = atan2(accelY, accelZ) * 180.0 / M_PI;
        float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
        
        // Gyroscope-based angle integration (velocity -> position)
        float gyroRollRate = gyroX / G_CONVERT;      // deg/s
        float gyroPitchRate = gyroY / G_CONVERT;     // deg/s
        float gyroYawRate = gyroZ / G_CONVERT;       // deg/s
        
        // Complementary filter: Fuse accelerometer and gyroscope
        // Gyroscope is fast but drifts, accelerometer is slow but accurate
        roll = ALPHA * (roll + gyroRollRate * dt) + (1 - ALPHA) * accelRoll;
        pitch = ALPHA * (pitch + gyroPitchRate * dt) + (1 - ALPHA) * accelPitch;
        yaw = yaw + gyroYawRate * dt;  // Yaw has no accelerometer reference
        
        // Constrain angles to -180 to 180 degrees
        roll = constrainAngle(roll);
        pitch = constrainAngle(pitch);
        yaw = constrainAngle(yaw);
        
        // Send JSON formatted data over serial
        // Python script expects this exact format
        Serial.print("{\"roll\": ");
        Serial.print(roll, 2);  // 2 decimal places
        Serial.print(", \"pitch\": ");
        Serial.print(pitch, 2);
        Serial.print(", \"yaw\": ");
        Serial.print(yaw, 2);
        Serial.println("}");
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/*
 * Constrain angle to -180 to 180 degrees
 * This prevents angle wrap-around and keeps values in usable range
 */
float constrainAngle(float angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }
    while (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

// ============================================================================
// END OF CODE
// ============================================================================

/*
 * TROUBLESHOOTING GUIDE:
 * 
 * 1. "MPU6050 connection FAILED!":
 *    - Check wiring (SDA, SCL, VCC, GND)
 *    - Try increasing delay in Wire.begin() section
 *    - Verify I2C address (use I2C scanner if unsure)
 * 
 * 2. No serial output:
 *    - Check baud rate (115200)
 *    - Try different USB cable
 *    - Reset NodeMCU (press reset button)
 * 
 * 3. Data shows but values are constant:
 *    - Check if sensor is working (move it around)
 *    - Verify sensor is not in sleep mode
 * 
 * 4. Large angle drift:
 *    - This is normal due to gyroscope bias
 *    - Use calibration routine or increase ALPHA value
 * 
 * 5. Memory issues during compilation:
 *    - This code is optimized for ESP8266
 *    - Remove unnecessary Serial.println() if needed
 */
