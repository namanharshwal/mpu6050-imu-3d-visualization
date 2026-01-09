#!/usr/bin/env python3
"""
Arduino Uno MPU6050 Simulator with 3D Visualization Window
Simulates MPU6050 sensor data with real-time 3D visualization
For testing and development without hardware

Author: Naman Harshwal
License: MIT
"""

import json
import time
import math
import random
import sys
import threading
from datetime import datetime

try:
    import pygame
    from pygame.locals import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("[WARNING] pygame/OpenGL not installed. Running in data-only mode.")
    print("[WARNING] Install with: pip3 install pygame PyOpenGL")

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

class Visualizer3D:
    def __init__(self, simulator):
        """
        Initialize 3D visualization window
        """
        self.simulator = simulator
        self.running = True
        
        if not PYGAME_AVAILABLE:
            return
        
        # Initialize Pygame and OpenGL
        pygame.init()
        display = (800, 600)
        self.screen = pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("MPU6050 3D Simulator - Arduino Uno")
        
        # OpenGL initialization
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5)
        
        # Lighting setup
        glLight(GL_LIGHT0, GL_POSITION, (5, 5, 5, 0))
        glLight(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1))
        glLight(GL_LIGHT0, GL_DIFFUSE, (1, 1, 1, 1))
        glLight(GL_LIGHT0, GL_SPECULAR, (1, 1, 1, 1))
        
        self.clock = pygame.time.Clock()

    def draw_cube(self):
        """
        Draw a 3D cube representing the IMU device
        """
        glBegin(GL_QUADS)
        
        # Define cube with different colors
        # Front face - Red
        glColor3f(1, 0, 0)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        
        # Back face - Green
        glColor3f(0, 1, 0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        
        # Top face - Blue
        glColor3f(0, 0, 1)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, -0.5)
        
        # Bottom face - Yellow
        glColor3f(1, 1, 0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        
        # Right face - Cyan
        glColor3f(0, 1, 1)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        
        # Left face - Magenta
        glColor3f(1, 0, 1)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        
        glEnd()

    def render(self):
        """
        Render the 3D visualization
        """
        if not PYGAME_AVAILABLE:
            return
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glClearColor(0.1, 0.1, 0.1, 1)
        
        glPushMatrix()
        
        # Apply rotation based on sensor data
        glRotatef(self.simulator.roll, 1, 0, 0)
        glRotatef(self.simulator.pitch, 0, 1, 0)
        glRotatef(self.simulator.yaw, 0, 0, 1)
        
        # Draw the cube
        self.draw_cube()
        
        glPopMatrix()
        
        pygame.display.flip()
        self.clock.tick(60)  # 60 FPS

    def update(self):
        """
        Handle events
        """
        if not PYGAME_AVAILABLE:
            return
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False

    def close(self):
        """
        Close the visualization window
        """
        if PYGAME_AVAILABLE:
            pygame.quit()

def visualization_thread(visualizer, simulator):
    """
    Thread for running visualization
    """
    if not PYGAME_AVAILABLE:
        return
    
    while visualizer.running:
        visualizer.update()
        visualizer.render()

def main():
    """
    Main simulator loop with optional 3D visualization
    """
    print("[MPU6050 Simulator] Starting Arduino Uno Simulator with 3D Visualization...")
    print("[MPU6050 Simulator] Simulating sensor data at 10Hz")
    print("[MPU6050 Simulator] Press Ctrl+C to stop\n")
    
    if PYGAME_AVAILABLE:
        print("[MPU6050 Simulator] 3D Visualization ENABLED")
    else:
        print("[MPU6050 Simulator] 3D Visualization DISABLED (pygame/OpenGL required)")
        print("[MPU6050 Simulator] Data will be output to console only\n")
    
    simulator = MPU6050Simulator(sampling_rate=10)
    visualizer = Visualizer3D(simulator)
    
    # Start visualization thread if available
    if PYGAME_AVAILABLE:
        vis_thread = threading.Thread(target=visualization_thread, args=(visualizer, simulator))
        vis_thread.daemon = True
        vis_thread.start()
    
    start_time = time.time()
    
    try:
        while True:
            # For visualization: check if window still running
            if PYGAME_AVAILABLE and not visualizer.running:
                break
            
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
        if PYGAME_AVAILABLE:
            visualizer.running = False
            visualizer.close()
        print("[MPU6050 Simulator] Simulator stopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()
