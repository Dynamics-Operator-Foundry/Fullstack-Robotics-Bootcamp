import mujoco
import mujoco.viewer
import numpy as np
import time
import socket
import struct
import select
import csv
import os
from sympy import *
import sys
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../kinematics/script/')))
print(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../kinematics/')))
from kinematics import kinematics


PORT = 60003
BUFFER_SIZE = 1024
fmt = 'dddd'  # 4 doubles

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', PORT))  # bind to all interfaces

model = mujoco.MjModel.from_xml_path("./rockie/rockie.xml")
data = mujoco.MjData(model)
# ee_site_id = model.site("ee_marker").id


sock.setblocking(False)

kine = kinematics()

csv_filename = "../log/joint_data_log.csv"
if os.path.exists(csv_filename):
    os.remove(csv_filename)
    
# with open(csv_filename, mode='w', newline='') as csvfile:
#     csv_writer = csv.writer(csvfile)
#     csv_writer.writerow(["time", "joint1_deg", "joint2_deg", "joint3_deg", "ee_x", "ee_y", "ee_z"])  # header

def set_joint_controls(time, data, target_angles, csv_writer, csvfile):
    """Set actuator control signals instead of directly setting positions"""
    # Set control signals for the 4 actuators
    for i, angle in enumerate(target_angles):
        data.ctrl[i] = angle
    
    # csv recording
    csv_writer.writerow([time, target_angles[0], target_angles[1], target_angles[2], target_angles[3], 0, 0, 0])
    csvfile.flush()

def generate_walking_gait(t):
    """Generate sine wave walking pattern for 4 joints (2 legs with hip and ankle each)"""
    # frequency = 0.5  # Hz - very slow walking
    # hip_amplitude = 0.3  # radians - minimal hip swing
    # ankle_amplitude = 0.2  # radians - minimal ankle bend
    # ankle_bias = -0.05  # very slight forward lean at ankles
    
    
    frequency = 0.5
    hip_amplitude = 0.25
    ankle_amplitude = 0.25
    ankle_bias = -0.10
    
    # Left leg (joint_1 = hip, joint_2 = ankle)
    left_hip = hip_amplitude * np.sin(2 * np.pi * frequency * t)
    left_ankle = ankle_bias + ankle_amplitude * np.sin(2 * np.pi * frequency * t + np.pi/6)
    
    # Right leg (joint_3 = hip, joint_4 = ankle) - opposite phase
    right_hip = hip_amplitude * np.sin(2 * np.pi * frequency * t + np.pi)
    right_ankle = ankle_bias + ankle_amplitude * np.sin(2 * np.pi * frequency * t + np.pi + np.pi/6)
    
    return [left_hip, left_ankle, right_hip, right_ankle] 

def sim():
    with open(csv_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["time", "joint1_rad", "joint2_rad", "joint3_rad", "joint4_rad", "ee_x", "ee_y", "ee_z"])  # header

        # Setup video recording
        video_filename = "../log/walking_robot.mp4"
        fps = 30
        width, height = 1280, 720
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        
        # Create renderer for video (separate from viewer)
        renderer = mujoco.Renderer(model, height=height, width=width)
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set viewer camera
            viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.1])     
            viewer.cam.distance = 0.8
            viewer.cam.azimuth = -90
            viewer.cam.elevation = -20

            # Set recording camera (side view)
            camera = mujoco.MjvCamera()
            camera.lookat = np.array([0.0, 0.0, 0.08])
            camera.distance = 0.6
            camera.azimuth = 90
            camera.elevation = -15

            start_time = time.time()
            last_frame_capture = 0
            frame_interval = 1.0 / fps  # 0.033s for 30fps
            duration = 200  # 10 second recording
            
            # Get base body ID for tracking
            base_body_id = model.body("base").id
            
            while viewer.is_running():
                current_time = time.time() - start_time
                
                # Stop recording after duration
                if current_time > duration:
                    break
                
                # Generate walking gait using sine waves
                target_angles = generate_walking_gait(current_time)
                
                # Set control signals (not direct positions)
                set_joint_controls(current_time, data, target_angles, csv_writer, csvfile)
                
                # Step the physics simulation (this runs collision detection, gravity, etc.)
                mujoco.mj_step(model, data)
                
                # Update camera to follow robot
                base_pos = data.xpos[base_body_id]
                viewer.cam.lookat[:] = base_pos
                camera.lookat = base_pos.copy()
                
                # Capture video frame at target FPS
                if current_time - last_frame_capture >= frame_interval:
                    renderer.update_scene(data, camera)
                    pixels = renderer.render()
                    frame = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                    video_writer.write(frame)
                    last_frame_capture = current_time
                
                # Sync viewer
                viewer.sync()
                time.sleep(0.01)  # 100 Hz update rate
        
        # Cleanup
        video_writer.release()
        renderer.close()
        print(f"Video saved to {os.path.abspath(video_filename)}")
    

if __name__ == "__main__":
    sim()