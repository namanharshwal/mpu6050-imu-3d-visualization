#!/usr/bin/env python3
"""
Arduino Uno MPU6050 Simulator
Simulates MPU6050 sensor data and outputs it in the same format as the actual Arduino firmware
For testing and development without hardware

Author: Naman Harshwal
License: MIT
"""

import json
import time
import math
import random
import sys
from datetime import datetime

class MPU6050Simulator:
    def __init__(self, sampling_rate=10):
        """
        Initialize simulator
        sampling_rate: Hz (default 10)
        """
        self.sampling_rate = sampling_rate
        self.interval = 1.0 / sampling_rate
        
        # Simulated sensor states
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81
        
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        self.temp = 35.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.timestamp = 0

    def simulate_motion(self, time_step):
        """
        Simulate realistic sensor motion patterns
        """
        # Simulate slow rotation
        self.roll += math.sin(time_step * 0.1) * 0.5
        self.pitch += math.cos(time_step * 0.1) * 0.5
        self.yaw += 0.2
        
        # Keep angles within bounds
        self.roll = (self.roll + 180) % 360 - 180
        self.pitch = (self.pitch + 180) % 360 - 180
        self.yaw = self.yaw % 360
        
        # Simulate accelerometer based on orientation
        self.accel_x = 9.81 * math.sin(math.radians(self.roll))
        self.accel_y = 9.81 * math.sin(math.radians(self.pitch))
        self.accel_z = 9.81 * math.cos(math.radians(self.roll)) * math.cos(math.radians(self.pitch))
        
        # Add small noise
        self.accel_x += random.uniform(-0.1, 0.1)
        self.accel_y += random.uniform(-0.1, 0.1)
        self.accel_z += random.uniform(-0.1, 0.1)
        
        # Simulate gyroscope
        self.gyro_x = math.sin(time_step * 0.05) * 2
        self.gyro_y = math.cos(time_step * 0.05) * 2
        self.gyro_z = 0.2 + random.uniform(-0.01, 0.01)
        
        # Simulate temperature variations
        self.temp = 35.0 + math.sin(time_step * 0.01) * 2 + random.uniform(-0.5, 0.5)

    def get_sensor_data(self):
        """
        Return sensor data in JSON format matching Arduino firmware output
        """
        data = {
            "timestamp": self.timestamp,
            "accel_x": round(self.accel_x, 4),
            "accel_y": round(self.accel_y, 4),
            "accel_z": round(self.accel_z, 4),
            "gyro_x": round(self.gyro_x, 6),
            "gyro_y": round(self.gyro_y, 6),
            "gyro_z": round(self.gyro_z, 6),
            "roll": round(self.roll, 2),
            "pitch": round(self.pitch, 2),
            "yaw": round(self.yaw, 2),
            "temp": round(self.temp, 2)
        }
        return data

def main():
    """
    Main simulator loop - outputs sensor data
    """
    print("[MPU6050 Simulator] Starting Arduino Uno Simulator...")
    print("[MPU6050 Simulator] Simulating sensor data at 10Hz")
    print("[MPU6050 Simulator] Press Ctrl+C to stop\n")
    
    simulator = MPU6050Simulator(sampling_rate=10)
    start_time = time.time()
    
    try:
        while True:
            elapsed = time.time() - start_time
            simulator.timestamp = int(elapsed * 1000)
            
            # Simulate motion
            simulator.simulate_motion(elapsed)
            
            # Get and print data
            data = simulator.get_sensor_data()
            print(json.dumps(data))
            sys.stdout.flush()
            
            # Sleep for sampling interval
            time.sleep(simulator.interval)
            
    except KeyboardInterrupt:
        print("\n[MPU6050 Simulator] Shutdown requested...")
        print("[MPU6050 Simulator] Simulator stopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()
