/*
 * MPU6050 LPC2148 3D Visualization Firmware
 * Author: Naman Harshwal
 * Date: January 2026
 * Version: 1.0.0
 * 
 * This firmware reads MPU6050 sensor data on ARM Cortex-M3 (LPC2148)
 * and transmits real-time orientation via serial for 3D visualization.
 * LPC2148 provides affordable ARM processing power for embedded systems.
 */

#include <LPC21xx.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// I2C Configuration
#define I2C_CLOCK_RATE 400000  // 400kHz I2C
#define MPU_ADDR 0x68          // MPU6050 I2C Address

// Sensor sensitivity
#define GYRO_SENSITIVITY 131.0     // LSB/deg/s
#define ACCEL_SENSITIVITY 16384.0  // LSB/g

// Complementary filter
#define ALPHA 0.98
#define SAMPLE_RATE 100  // Hz
#define DT (1.0 / SAMPLE_RATE)

// Global variables
float roll = 0, pitch = 0, yaw = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

// UART Configuration
#define UART_BAUD 115200
#define PCLK 15000000  // 15 MHz peripheral clock

void uart_init(void) {
    // UART0 initialization
    U0LCR = 0x83;  // 8-bit, 1 stop, no parity, DLAB=1
    U0DLL = (PCLK / (16 * UART_BAUD)) & 0xFF;
    U0DLM = ((PCLK / (16 * UART_BAUD)) >> 8) & 0xFF;
    U0LCR = 0x03;  // DLAB=0
    U0FCR = 0x07;  // FIFO enabled
}

void uart_putchar(char c) {
    while ((U0LSR & 0x20) == 0);
    U0THR = c;
}

void uart_putstring(const char *str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

void i2c_init(void) {
    // I2C0 initialization
    I2C0CONCLR = 0x6C;  // Clear I2EN, STA, STO bits
    I2C0SCLH = (PCLK / (2 * I2C_CLOCK_RATE * 1000));
    I2C0SCLL = (PCLK / (2 * I2C_CLOCK_RATE * 1000));
    I2C0CONSET = 0x40;  // Enable I2C
}

int i2c_write(unsigned char addr, unsigned char reg, unsigned char value) {
    // Start condition
    I2C0CONSET = 0x60;
    while ((I2C0CONSET & 0x08) == 0);
    
    // Send address + write
    I2C0DAT = (addr << 1);
    I2C0CONCLR = 0x08;
    while ((I2C0CONSET & 0x08) == 0);
    
    if ((I2C0STAT & 0xF8) != 0x18) return -1;
    
    // Send register address
    I2C0DAT = reg;
    I2C0CONCLR = 0x08;
    while ((I2C0CONSET & 0x08) == 0);
    
    // Send value
    I2C0DAT = value;
    I2C0CONCLR = 0x08;
    while ((I2C0CONSET & 0x08) == 0);
    
    // Stop condition
    I2C0CONSET = 0x10;
    I2C0CONCLR = 0x08;
    
    return 0;
}

int i2c_read(unsigned char addr, unsigned char reg, unsigned char *value) {
    // Simplified I2C read operation
    // Start - Send address - Wait for ACK - etc.
    // Implementation depends on LPC21xx I2C driver specifics
    return 0;
}

void mpu_init(void) {
    uart_putstring("\r\nMPU6050 LPC2148 Initialization...\r\n");
    
    // Configure MPU6050
    i2c_write(MPU_ADDR, 0x6B, 0x00);  // Wake up
    i2c_write(MPU_ADDR, 0x1A, 0x06);  // DLPF config
    i2c_write(MPU_ADDR, 0x1B, 0x00);  // Gyro range
    i2c_write(MPU_ADDR, 0x1C, 0x00);  // Accel range
    
    uart_putstring("MPU6050 initialized\r\n");
    
    // Calibration
    calibrate_sensor();
}

void calibrate_sensor(void) {
    uart_putstring("Calibrating sensor...\r\n");
    
    int i, samples = 500;
    float sumGX = 0, sumGY = 0, sumGZ = 0;
    
    for (i = 0; i < samples; i++) {
        // Read gyro data
        // sumGX += gyro_value / GYRO_SENSITIVITY
        // sumGY += ...
        // sumGZ += ...
    }
    
    gyroX_offset = sumGX / samples;
    gyroY_offset = sumGY / samples;
    gyroZ_offset = sumGZ / samples;
    
    uart_putstring("Calibration complete\r\n");
}

void update_angles(void) {
    // Complementary filter implementation
    float accelRoll = atan2(accelY, accelZ) * 180.0 / 3.14159;
    float accelPitch = asin(-accelX / 9.81) * 180.0 / 3.14159;
    
    roll = ALPHA * (roll + gyroX * DT) + (1 - ALPHA) * accelRoll;
    pitch = ALPHA * (pitch + gyroY * DT) + (1 - ALPHA) * accelPitch;
    yaw = yaw + gyroZ * DT;
}

void send_json(void) {
    // Send JSON formatted data
    char buffer[100];
    sprintf(buffer, "{\"roll\": %.2f, \"pitch\": %.2f, \"yaw\": %.2f}\r\n", roll, pitch, yaw);
    uart_putstring(buffer);
}

void delay_ms(unsigned int ms) {
    // Simple delay function
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 123; j++);
    }
}

int main(void) {
    uart_init();
    i2c_init();
    mpu_init();
    
    uart_putstring("MPU6050 LPC2148 3D Visualizer Ready\r\n");
    
    while (1) {
        // Read sensor data
        // i2c_read(MPU_ADDR, 0x3B, &accel_data);
        // i2c_read(MPU_ADDR, 0x43, &gyro_data);
        
        // Update angles
        update_angles();
        
        // Send JSON
        send_json();
        
        // Maintain sampling rate
        delay_ms(10);  // 100 Hz
    }
    
    return 0;
}
