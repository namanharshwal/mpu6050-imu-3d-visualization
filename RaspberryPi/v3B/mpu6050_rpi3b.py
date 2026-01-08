#!/usr/bin/env python3
"""
MPU6050 3D Visualization for Raspberry Pi 3B
Author: Naman Harshwal
Version: 1.0.0
License: MIT
Date: January 2026

Real-time IMU sensor data acquisition and processing with complementary filter fusion.
"""

import smbus
import time
import math
import json
import sys
import struct
from datetime import datetime

# I2C Configuration
I2C_BUS = 1  # Raspberry Pi 3B uses I2C bus 1
MPU6050_ADDR = 0x68  # Default MPU6050 address (AD0 = GND)

# MPU6050 Register Addresses
MPU6050_REG_POWER = 0x6B  # Power management register
MPU6050_REG_ACCEL_CONFIG = 0x1C  # Accelerometer configuration
MPU6050_REG_GYRO_CONFIG = 0x1B  # Gyroscope configuration
MPU6050_REG_ACCEL_XOUT_H = 0x3B  # Accelerometer X-axis data (MSB)
MPU6050_REG_GYRO_XOUT_H = 0x43  # Gyroscope X-axis data (MSB)
MPU6050_REG_TEMP_OUT_H = 0x41  # Temperature data (MSB)

# Scale factors for data conversion
ACCEL_SCALE = 16384.0  # For ±2g range
GYRO_SCALE = 131.0  # For ±250 deg/s range

# Complementary filter coefficients
ALPHA = 0.98  # Gyroscope weight (98% gyro, 2% accel)
DT = 0.1  # Time delta (100ms = 10Hz sampling)

class MPU6050:
    """
    Complete MPU6050 sensor interface for Raspberry Pi 3B
    Implements complementary filter for accurate orientation estimation
    """
    
    def __init__(self, bus=I2C_BUS, addr=MPU6050_ADDR):
        """
        Initialize MPU6050 sensor
        
        Args:
            bus: I2C bus number (1 for RPi 3B)
            addr: I2C address of MPU6050
        """
        try:
            self.bus = smbus.SMBus(bus)
            self.addr = addr
            self.initialized = False
            
            # Initialize orientation angles
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            
            # Test connection
            self._test_connection()
            
            # Configure sensor
            self._configure_sensor()
            
            self.initialized = True
            print(f"[SUCCESS] MPU6050 initialized at address 0x{self.addr:02X}")
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize MPU6050: {e}")
            sys.exit(1)
    
    def _test_connection(self):
        """
        Test connection to MPU6050 by reading WHO_AM_I register
        """
        try:
            who_am_i = self.bus.read_byte_data(self.addr, 0x75)
            if who_am_i != 0x68:
                raise ValueError(f"Invalid WHO_AM_I response: 0x{who_am_i:02X}")
            print("[INFO] MPU6050 connection verified")
        except IOError as e:
            print(f"[ERROR] I2C communication failed: {e}")
            raise
    
    def _configure_sensor(self):
        """
        Configure MPU6050 sensor registers
        - Power up the sensor
        - Set accelerometer range to ±2g
        - Set gyroscope range to ±250 deg/s
        - Disable SLEEP mode
        """
        try:
            # Wake up MPU6050 (clear sleep bit)
            self.bus.write_byte_data(self.addr, MPU6050_REG_POWER, 0x00)
            time.sleep(0.1)
            
            # Set accelerometer range to ±2g (0x00)
            self.bus.write_byte_data(self.addr, MPU6050_REG_ACCEL_CONFIG, 0x00)
            
            # Set gyroscope range to ±250 deg/s (0x00)
            self.bus.write_byte_data(self.addr, MPU6050_REG_GYRO_CONFIG, 0x00)
            
            time.sleep(0.1)
            print("[INFO] Sensor configuration complete")
            
        except IOError as e:
            print(f"[ERROR] Failed to configure sensor: {e}")
            raise
    
    def _read_raw_data(self, reg):
        """
        Read raw 16-bit signed data from sensor register
        
        Args:
            reg: Register address
            
        Returns:
            16-bit signed integer value
        """
        try:
            high = self.bus.read_byte_data(self.addr, reg)
            low = self.bus.read_byte_data(self.addr, reg + 1)
            
            # Combine high and low bytes
            value = (high << 8) | low
            
            # Convert to signed integer
            if value & 0x8000:
                value = -(65536 - value)
            
            return value
            
        except IOError as e:
            print(f"[ERROR] Failed to read data from register 0x{reg:02X}: {e}")
            return 0
    
    def read_accel(self):
        """
        Read acceleration data from all three axes
        
        Returns:
            Tuple of (ax, ay, az) in g's
        """
        ax_raw = self._read_raw_data(MPU6050_REG_ACCEL_XOUT_H)
        ay_raw = self._read_raw_data(0x3D)  # ACCEL_YOUT_H
        az_raw = self._read_raw_data(0x3F)  # ACCEL_ZOUT_H
        
        # Convert to g's
        ax = ax_raw / ACCEL_SCALE
        ay = ay_raw / ACCEL_SCALE
        az = az_raw / ACCEL_SCALE
        
        return ax, ay, az
    
    def read_gyro(self):
        """
        Read gyroscope data from all three axes
        
        Returns:
            Tuple of (gx, gy, gz) in deg/s
        """
        gx_raw = self._read_raw_data(MPU6050_REG_GYRO_XOUT_H)
        gy_raw = self._read_raw_data(0x45)  # GYRO_YOUT_H
        gz_raw = self._read_raw_data(0x47)  # GYRO_ZOUT_H
        
        # Convert to deg/s
        gx = gx_raw / GYRO_SCALE
        gy = gy_raw / GYRO_SCALE
        gz = gz_raw / GYRO_SCALE
        
        return gx, gy, gz
    
    def read_temp(self):
        """
        Read temperature sensor data
        
        Returns:
            Temperature in Celsius
        """
        temp_raw = self._read_raw_data(MPU6050_REG_TEMP_OUT_H)
        temp_c = (temp_raw / 340.0) + 36.53  # From MPU6050 datasheet
        return temp_c
    
    def update_orientation(self):
        """
        Calculate orientation angles using complementary filter
        Combines gyroscope (fast) and accelerometer (accurate) data
        
        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        # Read sensor data
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()
        
        # Calculate accelerometer angles (in degrees)
        accel_roll = math.atan2(ay, az) * 180.0 / math.pi
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180.0 / math.pi
        
        # Complementary filter: Fuse gyroscope and accelerometer
        # Gyroscope provides fast updates but drifts over time
        # Accelerometer provides accurate reference but is noisy
        self.roll = ALPHA * (self.roll + gx * DT) + (1 - ALPHA) * accel_roll
        self.pitch = ALPHA * (self.pitch + gy * DT) + (1 - ALPHA) * accel_pitch
        self.yaw = self.yaw + gz * DT  # Yaw has no accelerometer reference
        
        # Constrain angles to [-180, 180] degrees
        self.roll = self._constrain_angle(self.roll)
        self.pitch = self._constrain_angle(self.pitch)
        self.yaw = self._constrain_angle(self.yaw)
        
        return self.roll, self.pitch, self.yaw
    
    @staticmethod
    def _constrain_angle(angle):
        """
        Constrain angle to [-180, 180] degrees
        Prevents angle wrap-around and keeps values in usable range
        """
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle
    
    def get_json_output(self):
        """
        Get sensor data as JSON formatted string
        Compatible with Python visualization scripts
        
        Returns:
            JSON string with roll, pitch, yaw
        """
        self.update_orientation()
        data = {
            "roll": round(self.roll, 2),
            "pitch": round(self.pitch, 2),
            "yaw": round(self.yaw, 2),
            "timestamp": datetime.now().isoformat()
        }
        return json.dumps(data)
    
    def close(self):
        """
        Properly close I2C connection
        """
        try:
            self.bus.close()
            print("[INFO] I2C connection closed")
        except Exception as e:
            print(f"[ERROR] Failed to close I2C: {e}")


def main():
    """
    Main function - continuously read and output sensor data
    """
    sensor = None
    try:
        print("="*60)
        print(" MPU6050 3D Visualization - Raspberry Pi 3B")
        print(" Version: 1.0.0 | Author: Naman Harshwal")
        print(" License: MIT | Python Implementation")
        print("="*60)
        print()
        
        # Initialize sensor
        sensor = MPU6050()
        print(f"[INFO] Starting sensor data stream at 10Hz...")
        print(f"[INFO] Output format: JSON (roll, pitch, yaw)")
        print()
        
        # Main sensor loop
        while True:
            try:
                # Get formatted output
                json_output = sensor.get_json_output()
                print(json_output)
                
                # 10Hz sampling (100ms interval)
                time.sleep(DT)
                
            except KeyboardInterrupt:
                print("\n[INFO] Keyboard interrupt detected")
                break
            except Exception as e:
                print(f"[ERROR] Error in sensor loop: {e}")
                time.sleep(0.5)
                continue
    
    except Exception as e:
        print(f"[ERROR] Fatal error: {e}")
        sys.exit(1)
    
    finally:
        if sensor:
            sensor.close()
        print("[INFO] MPU6050 sensor disconnected")
        print("[INFO] Program terminated")


if __name__ == "__main__":
    main()
