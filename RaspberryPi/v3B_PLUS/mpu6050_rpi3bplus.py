#!/usr/bin/env python3
"""
MPU6050 3D IMU Visualization for Raspberry Pi 3B+
Author: Naman Harshwal
License: MIT

This script provides real-time 3D visualization of MPU6050 sensor data
using a complementary filter for sensor fusion on Raspberry Pi 3B+.
"""

import smbus
import math
import time
import json
import numpy as np
from threading import Thread
from collections import deque

# MPU6050 Registers and Constants
MPU6050_ADDR = 0x68
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_PWR_MGMT_2 = 0x6C
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_TEMP_OUT_H = 0x41
MPU6050_CONFIG = 0x1A
MPU6050_GYRO_CONFIG = 0x1B
MPU6050_ACCEL_CONFIG = 0x1C
MPU6050_INT_ENABLE = 0x38
MPU6050_WHO_AM_I = 0x75

# I2C Bus
BUS = smbus.SMBus(1)  # Raspberry Pi I2C bus 1


class MPU6050Sensor:
    """Complete MPU6050 sensor driver with sensor fusion"""
    
    def __init__(self, addr=MPU6050_ADDR, sample_rate=100):
        self.addr = addr
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        # Calibration values
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        
        # Current orientation (roll, pitch, yaw)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Complementary filter weights
        self.alpha = 0.95  # Gyroscope weight
        self.beta = 0.05   # Accelerometer weight
        
        # Data history for visualization
        self.data_history = deque(maxlen=1000)
        self.is_running = False
        
        self.initialize_sensor()
    
    def initialize_sensor(self):
        """Initialize MPU6050 sensor and verify connection"""
        try:
            # Check WHO_AM_I register
            who_am_i = self.read_byte(MPU6050_WHO_AM_I)
            if who_am_i != 0x68:
                raise Exception(f"MPU6050 not found! WHO_AM_I returned: {hex(who_am_i)}")
            
            print(f"\n{'='*50}")
            print("MPU6050 Sensor Initialization")
            print(f"{'='*50}")
            print(f"I2C Address: 0x{self.addr:02X}")
            print(f"Sensor ID (WHO_AM_I): 0x{who_am_i:02X}")
            
            # Wake up the sensor
            self.write_byte(MPU6050_PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # Set gyroscope and accelerometer ranges
            self.write_byte(MPU6050_GYRO_CONFIG, 0x00)  # ±250°/s
            self.write_byte(MPU6050_ACCEL_CONFIG, 0x00)  # ±2g
            
            # Set digital low-pass filter
            self.write_byte(MPU6050_CONFIG, 0x06)  # 5Hz bandwidth
            
            # Enable interrupt
            self.write_byte(MPU6050_INT_ENABLE, 0x01)
            
            print(f"Gyroscope range: ±250°/s")
            print(f"Accelerometer range: ±2g")
            print(f"Sample rate: {self.sample_rate} Hz")
            print(f"DLPF bandwidth: 5 Hz")
            print(f"{'='*50}\n")
            
            # Calibrate sensors
            self.calibrate_sensors()
            
        except Exception as e:
            print(f"Sensor Initialization Error: {e}")
            raise
    
    def calibrate_sensors(self, samples=100):
        """Calibrate accelerometer and gyroscope offsets"""
        print("Calibrating sensors... Keep the device still and level.")
        
        accel_sum = np.array([0.0, 0.0, 0.0])
        gyro_sum = np.array([0.0, 0.0, 0.0])
        
        for _ in range(samples):
            accel_data = self.read_accel()
            gyro_data = self.read_gyro()
            
            accel_sum += accel_data
            gyro_sum += gyro_data
            
            time.sleep(self.dt)
        
        self.accel_offset = accel_sum / samples
        self.gyro_offset = gyro_sum / samples
        
        # Subtract gravity from Z-axis
        self.accel_offset[2] -= 9.81
        
        print(f"Accelerometer offset: {self.accel_offset}")
        print(f"Gyroscope offset: {self.gyro_offset}")
        print("Calibration complete!\n")
    
    def read_byte(self, register):
        """Read single byte from register"""
        return BUS.read_byte_data(self.addr, register)
    
    def write_byte(self, register, value):
        """Write single byte to register"""
        BUS.write_byte_data(self.addr, register, value)
    
    def read_word(self, register):
        """Read 16-bit word from register (MSB first)"""
        high = self.read_byte(register)
        low = self.read_byte(register + 1)
        value = (high << 8) | low
        
        # Handle signed values
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        
        return value
    
    def read_accel(self):
        """Read accelerometer data in m/s²"""
        ax_raw = self.read_word(MPU6050_ACCEL_XOUT_H) / 16384.0 * 9.81  # Scale to ±2g
        ay_raw = self.read_word(MPU6050_ACCEL_XOUT_H + 2) / 16384.0 * 9.81
        az_raw = self.read_word(MPU6050_ACCEL_XOUT_H + 4) / 16384.0 * 9.81
        
        return np.array([ax_raw, ay_raw, az_raw]) - self.accel_offset
    
    def read_gyro(self):
        """Read gyroscope data in degrees/second"""
        gx_raw = self.read_word(MPU6050_GYRO_XOUT_H) / 131.0  # Scale to ±250°/s
        gy_raw = self.read_word(MPU6050_GYRO_XOUT_H + 2) / 131.0
        gz_raw = self.read_word(MPU6050_GYRO_XOUT_H + 4) / 131.0
        
        return np.array([gx_raw, gy_raw, gz_raw]) - self.gyro_offset
    
    def read_temp(self):
        """Read temperature in Celsius"""
        raw_temp = self.read_word(MPU6050_TEMP_OUT_H)
        temp_c = raw_temp / 340.0 + 36.53
        return temp_c
    
    def complementary_filter(self, accel, gyro):
        """Apply complementary filter for sensor fusion"""
        # Calculate angles from accelerometer
        accel_roll = math.atan2(accel[1], accel[2]) * 180.0 / math.pi
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / math.pi
        
        # Apply complementary filter
        self.roll = self.alpha * (self.roll + gyro[0] * self.dt) + self.beta * accel_roll
        self.pitch = self.alpha * (self.pitch + gyro[1] * self.dt) + self.beta * accel_pitch
        self.yaw += gyro[2] * self.dt
    
    def read_sensor(self):
        """Read all sensor data and update orientation"""
        try:
            accel = self.read_accel()
            gyro = self.read_gyro()
            temp = self.read_temp()
            
            # Update orientation using complementary filter
            self.complementary_filter(accel, gyro)
            
            # Create data record
            data = {
                'timestamp': time.time(),
                'accel_x': round(accel[0], 4),
                'accel_y': round(accel[1], 4),
                'accel_z': round(accel[2], 4),
                'gyro_x': round(gyro[0], 4),
                'gyro_y': round(gyro[1], 4),
                'gyro_z': round(gyro[2], 4),
                'roll': round(self.roll, 2),
                'pitch': round(self.pitch, 2),
                'yaw': round(self.yaw, 2),
                'temp': round(temp, 2)
            }
            
            self.data_history.append(data)
            return data
            
        except Exception as e:
            print(f"Sensor Read Error: {e}")
            return None
    
    def start_continuous_reading(self):
        """Start continuous sensor reading in background thread"""
        self.is_running = True
        
        def read_loop():
            while self.is_running:
                self.read_sensor()
                time.sleep(self.dt)
        
        thread = Thread(target=read_loop, daemon=True)
        thread.start()
        print("Continuous sensor reading started.")
    
    def stop_continuous_reading(self):
        """Stop continuous sensor reading"""
        self.is_running = False
        print("Continuous sensor reading stopped.")
    
    def get_latest_data(self):
        """Get the most recent sensor reading"""
        if self.data_history:
            return self.data_history[-1]
        return None
    
    def get_data_json(self):
        """Get latest sensor data as JSON string"""
        data = self.get_latest_data()
        if data:
            return json.dumps(data)
        return None


def main():
    """Main execution function with example usage"""
    print("\nMPU6050 3D IMU Visualization for Raspberry Pi 3B+")
    print("Author: Naman Harshwal")
    print("Repository: mpu6050-imu-3d-visualization\n")
    
    try:
        # Initialize sensor
        sensor = MPU6050Sensor(sample_rate=100)
        
        print("Starting 3D visualization...\n")
        print("Real-time Sensor Data:")
        print(f"{'Time':<12} {'Roll':<8} {'Pitch':<8} {'Yaw':<8} {'Temp':<8}")
        print("-" * 44)
        
        # Read sensor data for 30 seconds
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < 30:
            data = sensor.read_sensor()
            
            if data and count % 10 == 0:  # Print every 10th reading
                elapsed = time.time() - start_time
                print(f"{elapsed:<12.2f} {data['roll']:<8.2f} {data['pitch']:<8.2f} {data['yaw']:<8.2f} {data['temp']:<8.2f}")
            
            count += 1
            time.sleep(sensor.dt)
        
        print("-" * 44)
        print(f"Total readings: {count}")
        print(f"Average sample rate: {count / (time.time() - start_time):.2f} Hz")
        print("\nVisualization complete!\n")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Sensor cleanup complete.")


if __name__ == "__main__":
    main()
