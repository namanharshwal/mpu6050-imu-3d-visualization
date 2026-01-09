#!/usr/bin/env python3
"""
Arduino Uno MPU6050 3D Visualizer
Real-time 3D model visualization with serial IMU data from Arduino Uno

Author: Naman Harshwal
Date: January 2026
Version: 2.0.0

REQUIREMENTS:
    pip install numpy pygame PyOpenGL pyserial

USAGE:
    python Arduino_Uno_Visualizer.py
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import json
import sys
import time

# ============================================================================
# CONFIGURATION
# ============================================================================

# Serial Communication Configuration
SERIAL_PORT = 'COM3'          # Change to your COM port (COM3 for Arduino Uno)
BAUD_RATE = 115200            # Must match Arduino firmware
SERIAL_TIMEOUT = 1            # Timeout in seconds

# 3D Visualization Configuration
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
FPS_LIMIT = 60

# 3D Model Configuration
MODEL_SCALE = 0.5             # Adjust model size
MODEL_COLOR = (1.0, 0.0, 0.0) # Red color (R, G, B)

# Camera Configuration
CAMERA_DISTANCE = -50.0       # Distance from model

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

ser = None
roll = 0.0
pitch = 0.0
yaw = 0.0
temperature = 0.0

fps_clock = None
fps_counter = 0
last_fps_time = time.time()

# ============================================================================
# INITIALIZATION FUNCTIONS
# ============================================================================

def init_serial():
    """
    Initialize serial communication with Arduino Uno
    """
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        print(f"[SUCCESS] Serial connection established on {SERIAL_PORT}")
        print(f"[INFO] Baud rate: {BAUD_RATE}")
        time.sleep(2)  # Wait for Arduino to stabilize
        return True
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open serial port {SERIAL_PORT}")
        print(f"[ERROR] Details: {e}")
        print(f"[INFO] Check if Arduino Uno is connected and port is correct")
        return False

def init_opengl():
    """
    Initialize OpenGL settings
    """
    glShadeModel(GL_SMOOTH)
    glClearColor(0.1, 0.1, 0.1, 1.0)  # Dark gray background
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    
    # Lighting setup
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLight(GL_LIGHT0, GL_POSITION, (1.0, 1.0, 1.0, 0.0))
    glLight(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
    glLight(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))

def reshape(width, height):
    """
    Handle window resize
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, float(width) / float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def draw_axes():
    """
    Draw RGB axes for reference (X=Red, Y=Green, Z=Blue)
    """
    glDisable(GL_LIGHTING)
    glLineWidth(2.0)
    
    glBegin(GL_LINES)
    # X axis - Red
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(10, 0, 0)
    
    # Y axis - Green
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 10, 0)
    
    # Z axis - Blue
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 10)
    glEnd()
    
    glEnable(GL_LIGHTING)

def draw_cube():
    """
    Draw a reference cube (fallback if STL fails to load)
    """
    glColor3fv(MODEL_COLOR)
    size = 5
    
    glBegin(GL_QUADS)
    # Front
    glVertex3f(-size, -size, size)
    glVertex3f(size, -size, size)
    glVertex3f(size, size, size)
    glVertex3f(-size, size, size)
    
    # Back
    glVertex3f(-size, -size, -size)
    glVertex3f(-size, size, -size)
    glVertex3f(size, size, -size)
    glVertex3f(size, -size, -size)
    
    # Left
    glVertex3f(-size, -size, -size)
    glVertex3f(-size, -size, size)
    glVertex3f(-size, size, size)
    glVertex3f(-size, size, -size)
    
    # Right
    glVertex3f(size, -size, -size)
    glVertex3f(size, size, -size)
    glVertex3f(size, size, size)
    glVertex3f(size, -size, size)
    
    # Top
    glVertex3f(-size, size, -size)
    glVertex3f(-size, size, size)
    glVertex3f(size, size, size)
    glVertex3f(size, size, -size)
    
    # Bottom
    glVertex3f(-size, -size, -size)
    glVertex3f(size, -size, -size)
    glVertex3f(size, -size, size)
    glVertex3f(-size, -size, size)
    glEnd()

def display():
    """
    Render the 3D scene
    """
    global roll, pitch, yaw
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    
    # Position camera
    glTranslatef(0, 0, CAMERA_DISTANCE)
    
    # Apply rotations based on IMU data
    glRotatef(roll, 1.0, 0.0, 0.0)    # Roll around X-axis
    glRotatef(pitch, 0.0, 1.0, 0.0)   # Pitch around Y-axis
    glRotatef(yaw, 0.0, 0.0, 1.0)     # Yaw around Z-axis
    
    # Draw reference axes
    draw_axes()
    
    # Draw cube (simple fallback)
    glScalef(MODEL_SCALE, MODEL_SCALE, MODEL_SCALE)
    draw_cube()
    
    pygame.display.flip()

def read_serial_data():
    """
    Read and parse JSON data from Arduino Uno
    """
    global roll, pitch, yaw, temperature
    
    if ser is None or not ser.is_open:
        return
    
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line.startswith('{') and line.endswith('}'):
                try:
                    data = json.loads(line)
                    roll = float(data.get('roll', 0.0))
                    pitch = float(data.get('pitch', 0.0))
                    yaw = float(data.get('yaw', 0.0))
                    temperature = float(data.get('temp', 0.0))
                except json.JSONDecodeError:
                    pass  # Skip malformed JSON
    except Exception as e:
        print(f"[ERROR] Serial read error: {e}")

def update_fps():
    """
    Update and display FPS counter
    """
    global fps_counter, last_fps_time
    
    fps_counter += 1
    current_time = time.time()
    
    if current_time - last_fps_time >= 1.0:
        print(f"FPS: {fps_counter} | Roll: {roll:.2f}° | Pitch: {pitch:.2f}° | Yaw: {yaw:.2f}° | Temp: {temperature:.1f}°C")
        fps_counter = 0
        last_fps_time = current_time

def main():
    """
    Main function - Run the visualization
    """
    global fps_clock
    
    print("\n" + "="*60)
    print("  Arduino Uno - MPU6050 3D Visualizer")
    print("="*60)
    
    # Initialize serial
    if not init_serial():
        sys.exit(1)
    
    # Initialize pygame and OpenGL
    pygame.init()
    pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Arduino Uno MPU6050 - 3D Orientation Visualizer")
    
    reshape(WINDOW_WIDTH, WINDOW_HEIGHT)
    init_opengl()
    
    fps_clock = pygame.time.Clock()
    
    print("\n[INFO] Starting visualization...")
    print("[INFO] Window controls:")
    print("       - Close window to exit")
    print("       - Model rotates based on IMU data")
    print("       - Red/Green/Blue axes show X/Y/Z orientation")
    print("\n" + "="*60)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        # Read IMU data from Arduino Uno
        read_serial_data()
        
        # Render scene
        display()
        
        # Update FPS and limit frame rate
        update_fps()
        fps_clock.tick(FPS_LIMIT)
    
    # Cleanup
    if ser and ser.is_open:
        ser.close()
        print("\n[INFO] Serial connection closed")
    
    pygame.quit()
    print("[INFO] Visualization terminated")
    print("="*60 + "\n")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"\n[ERROR] Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
