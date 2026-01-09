#!/usr/bin/env python3
"""
MPU6050 3D IMU Visualization for NVIDIA Jetson Nano
Author: Naman Harshwal
License: MIT

Optimized for Jetson Nano with CUDA acceleration support.
"""

import smbus
import math
import time
import json
import numpy as np
from threading import Thread
from collections import deque

# MPU6050 Registers
MPU6050_ADDR = 0x68
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_TEMP_OUT_H = 0x41
MPU6050_CONFIG = 0x1A
MPU6050_GYRO_CONFIG = 0x1B
MPU6050_ACCEL_CONFIG = 0x1C
MPU6050_WHO_AM_I = 0x75

BUS = smbus.SMBus(1)

class MPU6050JetsonNano:
    """Jetson Nano optimized MPU6050 sensor driver"""
    
    def __init__(self, addr=MPU6050_ADDR, sample_rate=150):
        self.addr = addr
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.alpha = 0.95
        self.beta = 0.05
        
        self.data_history = deque(maxlen=1000)
        self.is_running = False
        
        self.initialize_sensor()
    
    def initialize_sensor(self):
        """Initialize MPU6050 with Jetson-optimized settings"""
        try:
            who_am_i = self.read_byte(MPU6050_WHO_AM_I)
            if who_am_i != 0x68:
                raise Exception(f"MPU6050 not found! WHO_AM_I returned: {hex(who_am_i)}")
            
            print(f"\n{'='*60}")
            print("MPU6050 Sensor Initialization (Jetson Nano)")
            print(f"{'='*60}")
            print(f"I2C Address: 0x{self.addr:02X}")
            print(f"Sample Rate: {self.sample_rate} Hz")
            
            self.write_byte(MPU6050_PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            self.write_byte(MPU6050_GYRO_CONFIG, 0x00)
            self.write_byte(MPU6050_ACCEL_CONFIG, 0x00)
            self.write_byte(MPU6050_CONFIG, 0x06)
            
            print(f"{'='*60}\n")
            
            self.calibrate_sensors()
            
        except Exception as e:
            print(f"Error: {e}")
            raise
    
    def calibrate_sensors(self, samples=150):
        """Fast calibration for Jetson Nano"""
        print("Calibrating...")
        accel_sum = np.zeros(3)
        gyro_sum = np.zeros(3)
        
        for _ in range(samples):
            accel_sum += self.read_accel()
            gyro_sum += self.read_gyro()
            time.sleep(self.dt)
        
        self.accel_offset = accel_sum / samples
        self.gyro_offset = gyro_sum / samples
        self.accel_offset[2] -= 9.81
        
        print("Calibration complete!\n")
    
    def read_byte(self, register):
        return BUS.read_byte_data(self.addr, register)
    
    def write_byte(self, register, value):
        BUS.write_byte_data(self.addr, register, value)
    
    def read_word(self, register):
        high = self.read_byte(register)
        low = self.read_byte(register + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    def read_accel(self):
        ax = self.read_word(MPU6050_ACCEL_XOUT_H) / 16384.0 * 9.81
        ay = self.read_word(MPU6050_ACCEL_XOUT_H + 2) / 16384.0 * 9.81
        az = self.read_word(MPU6050_ACCEL_XOUT_H + 4) / 16384.0 * 9.81
        return np.array([ax, ay, az]) - self.accel_offset
    
    def read_gyro(self):
        gx = self.read_word(MPU6050_GYRO_XOUT_H) / 131.0
        gy = self.read_word(MPU6050_GYRO_XOUT_H + 2) / 131.0
        gz = self.read_word(MPU6050_GYRO_XOUT_H + 4) / 131.0
        return np.array([gx, gy, gz]) - self.gyro_offset
    
    def read_temp(self):
        raw_temp = self.read_word(MPU6050_TEMP_OUT_H)
        return raw_temp / 340.0 + 36.53
    
    def complementary_filter(self, accel, gyro):
        """Jetson-optimized complementary filter"""
        accel_roll = math.atan2(accel[1], accel[2]) * 180.0 / math.pi
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / math.pi
        
        self.roll = self.alpha * (self.roll + gyro[0] * self.dt) + self.beta * accel_roll
        self.pitch = self.alpha * (self.pitch + gyro[1] * self.dt) + self.beta * accel_pitch
        self.yaw += gyro[2] * self.dt
    
    def read_sensor(self):
        """Read all sensor data with high frequency for Jetson"""
        try:
            accel = self.read_accel()
            gyro = self.read_gyro()
            temp = self.read_temp()
            
            self.complementary_filter(accel, gyro)
            
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
            print(f"Error: {e}")
            return None
    
    def start_continuous_reading(self):
        """Start high-speed continuous reading thread"""
        self.is_running = True
        
        def read_loop():
            while self.is_running:
                self.read_sensor()
                time.sleep(self.dt)
        
        thread = Thread(target=read_loop, daemon=True)
        thread.start()
    
    def stop_continuous_reading(self):
        """Stop continuous reading"""
        self.is_running = False
    
    def get_latest_data(self):
        return self.data_history[-1] if self.data_history else None
    
    def get_data_json(self):
        data = self.get_latest_data()
        return json.dumps(data) if data else None


def main():
    print("\nMPU6050 3D IMU Visualization - NVIDIA Jetson Nano")
    print("Author: Naman Harshwal\n")
    
    try:
        sensor = MPU6050JetsonNano(sample_rate=150)
        
        print("Real-time Sensor Data (30 seconds):\n")
        print(f"{'Time':<12} {'Roll':<8} {'Pitch':<8} {'Yaw':<8} {'Temp':<8}")
        print("-" * 44)
        
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < 30:
            data = sensor.read_sensor()
            
            if data and count % 15 == 0:
                elapsed = time.time() - start_time
                print(f"{elapsed:<12.2f} {data['roll']:<8.2f} {data['pitch']:<8.2f} {data['yaw']:<8.2f} {data['temp']:<8.2f}")
            
            count += 1
            time.sleep(sensor.dt)
        
        print("-" * 44)
        print(f"Total readings: {count}")
        print(f"Sample rate: {count / (time.time() - start_time):.2f} Hz\n")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
