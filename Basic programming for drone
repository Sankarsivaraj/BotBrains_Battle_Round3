from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading
import numpy as np
from rplidar import RPLidar
from colorsensor import ColorSensor
import socket

# Target properties and communication setup
TARGET_HEIGHT = 15  # How tall the target object is, in centimeters
TARGET_COLOR = 'green'  # The color we're looking for
SWARM_PORT = 5000  # Port for drone-to-drone chatting

# Connect to our drone
print("Connecting to the drone...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=57600) # baud rate can be modified 

# Get the sensors ready
lidar = RPLidar('/dev/ttyUSB0')  # LIDAR for seeing stuffs through grasslands 
color_sensor = ColorSensor()  # Color sensor for checking colors (green color)

# Get the communication setup for drone gossip
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_ip_list = ['192.168.1.101', '192.168.1.102', '192.168.1.103']  # Our drone buddies' IPs

def send_target_found():
    """Tell the other drones that we found the target."""
    message = "TARGET_FOUND"
    for ip in drone_ip_list:
        if ip != get_own_ip():  # Don't text ourselves
            sock.sendto(message.encode(), (ip, SWARM_PORT))
    print("Target found! Letting the other drones know.")

def receive_swarm_message():
    """Listen up for messages from our drone friends."""
    sock.bind((get_own_ip(), SWARM_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        if data.decode() == "TARGET_FOUND":
            print("Another drone found the target. Stopping our search.")
            stop_drone()
            break

def get_own_ip():
    """Get the drone's IP address."""
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    return ip_address

def stop_drone():
    """Make the drone stop and hover in place."""
    vehicle.mode = VehicleMode("LOITER")
    print("Hovering in place (LOITER mode activated).")

def start_mission():
    """Main function to search around and find the target."""
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if distance < 100:  # Only care about stuff closer than 100 cm
                height = distance * np.sin(np.deg2rad(angle))
                if abs(height - TARGET_HEIGHT) < 1:  # Checking if it's the height we want
                    print("Spotted something at the right height. Measuring more...")
                    dimensions = measure_dimensions(scan)
                    if check_target_dimensions(dimensions):
                        print("Dimensions match. Checking the color now...")
                        color = color_sensor.detect_color()
                        if color == TARGET_COLOR:
                            print("Found the target color! Notifying the swarm.")
                            send_target_found()
                            stop_drone()
                            return
                        else:
                            print("Color is not what we want. Continuing the search.")
        time.sleep(0.5)

def measure_dimensions(scan):
    """Measure the width and depth of what we found."""
    # Just dummy numbers for now
    width = np.random.uniform(10, 20)  # Pretend we're measuring width
    depth = np.random.uniform(10, 20)  # Pretend we're measuring depth
    print(f"Measured dimensions: width={width} cm, depth={depth} cm")
    return (width, depth)

def check_target_dimensions(dimensions):
    """See if the measured dimensions match our target dimensions."""
    width, depth = dimensions
    return abs(width - TARGET_HEIGHT) < 1 and abs(depth - TARGET_HEIGHT) < 1  # Allow some measurement error

def get_drone_positions():
    """Get the current spots of all the drones (stub function)."""
    # Placeholder for now, would normally get real data
    return [(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)] * 3  # Pretend positions

def get_own_position():
    """Get the drone's current location."""
    location = vehicle.location.global_frame
    return (location.lat, location.lon, location.alt)

# Start threads for the mission, avoiding crashes, and listening for messages
search_thread = threading.Thread(target=start_mission)
collision_thread = threading.Thread(target=avoid_collision)
communication_thread = threading.Thread(target=receive_swarm_message)

# Kick off the mission
search_thread.start()
collision_thread.start()
communication_thread.start()

# Keep the program running
search_thread.join()
collision_thread.join()
communication_thread.join()

# Clean up when done
vehicle.close()
lidar.stop()
sock.close()
