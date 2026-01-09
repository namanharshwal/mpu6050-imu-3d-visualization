import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import json
import time

# Setup serial communication for IMU data - Updated to 115200 baud
ser = serial.Serial('COM3', 115200, timeout=0.5)  # Adjust 'COM3' if needed

# IMU variables
ax = ay = az = 0.0
yaw_mode = False
packets_received = 0
packets_error = 0

# Function to initialize OpenGL
def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
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

# Function to resize the window
def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

# Function to draw a reference cube
def draw_cube():
    size = 5
    glColor3f(1.0, 0.0, 0.0) # Red color
    glBegin(GL_QUADS)
    
    # Front face
    glVertex3f(-size, -size, size)
    glVertex3f(size, -size, size)
    glVertex3f(size, size, size)
    glVertex3f(-size, size, size)
    
    # Back face
    glVertex3f(-size, -size, -size)
    glVertex3f(-size, size, -size)
    glVertex3f(size, size, -size)
    glVertex3f(size, -size, -size)
    
    # Left face
    glVertex3f(-size, -size, -size)
    glVertex3f(-size, -size, size)
    glVertex3f(-size, size, size)
    glVertex3f(-size, size, -size)
    
    # Right face
    glVertex3f(size, -size, -size)
    glVertex3f(size, size, -size)
    glVertex3f(size, size, size)
    glVertex3f(size, -size, size)
    
    # Top face
    glVertex3f(-size, size, -size)
    glVertex3f(-size, size, size)
    glVertex3f(size, size, size)
    glVertex3f(size, size, -size)
    
    # Bottom face
    glVertex3f(-size, -size, -size)
    glVertex3f(size, -size, -size)
    glVertex3f(size, -size, size)
    glVertex3f(-size, -size, size)
    
    glEnd()

# Function to draw the 3D model
def draw():
    global ax, ay, az, yaw_mode
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    
    # Move the camera further away
    glTranslatef(0, 0.0, -50.0)
    
    # Scale the model
    scaling_factor = 0.4
    glScalef(scaling_factor, scaling_factor, scaling_factor)
    
    # Rotate based on IMU data
    glRotatef(ay, 1.0, 0.0, 0.0) # Pitch, rotate around x-axis
    glRotatef(-1 * ax, 0.0, 0.0, 1.0) # Roll, rotate around z-axis
    
    if yaw_mode:
        glRotatef(az, 0.0, 1.0, 0.0) # Yaw, rotate around y-axis
    
    # Draw the 3D cube
    draw_cube()

# Function to read IMU data with robust error handling
def read_data():
    global ax, ay, az, packets_received, packets_error
    
    try:
        # Clear buffer of any garbage data
        if ser.in_waiting > 100:
            ser.reset_input_buffer()
        
        if ser.in_waiting > 0:
            # Read until newline for proper framing
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Validate JSON format
            if line.startswith("{") and line.endswith("}") and len(line) > 10:
                try:
                    imu_data = json.loads(line)  # Parse JSON
                    ax = float(imu_data.get("roll", ax))      # Extract roll (x-axis)
                    ay = float(imu_data.get("pitch", ay))     # Extract pitch (y-axis)
                    az = float(imu_data.get("yaw", az))       # Extract yaw (z-axis)
                    packets_received += 1
                except (json.JSONDecodeError, ValueError):
                    packets_error += 1
                    pass  # Skip corrupt data silently
            else:
                packets_error += 1
    except Exception as e:
        packets_error += 1
        pass  # Silent error handling

# Main function to run the program
def main():
    global yaw_mode, packets_received, packets_error
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Arduino Uno MPU6050 - 3D Orientation Visualizer")
    
    resize(640, 480)
    init() # Initialize OpenGL
    
    frames = 0
    ticks = pygame.time.get_ticks()
    
    print("[INFO] Arduino Uno MPU6050 3D Visualizer Started")
    print("[INFO] Serial: COM3 @ 115200 baud")
    print("[INFO] Press 'Z' to toggle Yaw mode")
    print("[INFO] Press ESC to exit\n")
    
    while True:
        event = pygame.event.poll()
        
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break
        
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            mode_str = "ENABLED" if yaw_mode else "DISABLED"
            print(f"[INFO] Yaw mode {mode_str}")
        
        # Read IMU data
        read_data()
        
        # Draw the 3D model
        draw()
        
        pygame.display.flip()
        
        frames += 1
        elapsed = pygame.time.get_ticks() - ticks
        if elapsed >= 1000:
            fps = frames * 1000 / elapsed
            packet_loss = (packets_error / (packets_received + packets_error) * 100) if (packets_received + packets_error) > 0 else 0
            print(f"[FPS] {fps:.1f} | Roll: {ax:7.2f}° | Pitch: {ay:7.2f}° | Yaw: {az:7.2f}° | Loss: {packet_loss:.1f}%")
            frames = 0
            packets_received = 0
            packets_error = 0
            ticks = pygame.time.get_ticks()
    
    ser.close()
    print("\n[INFO] Serial connection closed")
    print("[INFO] Visualizer stopped")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
