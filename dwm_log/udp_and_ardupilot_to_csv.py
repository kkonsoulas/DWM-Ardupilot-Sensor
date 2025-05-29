import socket
import subprocess
import os
import signal
import csv
import time
import threading
import re
from pymavlink import mavutil

# Configuration
CONNECTION_TYPE = "DRONE"  # Options: "SITL" or "DRONE"
SITL_CONN = "udp:127.0.0.1:14550"  # SITL connection
DRONE_CONN = "/dev/ttyUSB0,57600"  # Real drone serial connection (adjust as needed)
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
UPDATE_RATE_HZ = 10  # 10Hz for data collection
TIME_PERIOD = 1.0 / UPDATE_RATE_HZ
C_BINARY = "./dwm_udp_sender"  # Path to compiled C binary

# Global variables for latest rangefinder data
latest_alt = None
alt_lock = threading.Lock()

# Select MAVLink connection based on type
MAVLink_CONN = SITL_CONN if CONNECTION_TYPE == "SITL" else DRONE_CONN

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)  # 100ms timeout for non-blocking

# Bind UDP socket
try:
    sock.bind((UDP_IP, UDP_PORT))
except socket.error as e:
    print(f"Failed to bind UDP socket: {e}")
    exit(1)

# Start C binary
try:
    process = subprocess.Popen(C_BINARY, shell=False, preexec_fn=os.setsid)
    print(f"Started C binary: {C_BINARY}")
except Exception as e:
    print(f"Failed to start C binary: {e}")
    sock.close()
    exit(1)

# Connect to ArduPilot
try:
    master = mavutil.mavlink_connection(MAVLink_CONN)
    master.wait_heartbeat(timeout=5)
    print(f"Connected to ArduPilot ({CONNECTION_TYPE})")
except KeyboardInterrupt:
    print("Interrupted by user (Ctrl+C)")
    print(f"Sending SIGINT to PID: {process.pid}")
    os.kill(process.pid, signal.SIGINT)
    sock.close()
    exit(1)
except Exception as e:
    print(f"Failed to connect to ArduPilot: {e}")
    print(f"Sending SIGINT to PID: {process.pid}")
    os.kill(process.pid, signal.SIGINT)
    sock.close()
    exit(1)

# Request rangefinder data at 10Hz
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 100000, 0, 0, 0, 0, 0
)

# Function to collect rangefinder data
def collect_rangefinder():
    global latest_alt
    while True:
        msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=0.1)
        if msg:
            with alt_lock:
                latest_alt = msg.current_distance / 100.0  # cm to meters
                # print(f"Updated altitude: {latest_alt:.2f} m")
        # else:
            # print("No altitude message")
        time.sleep(0.01)  # Prevent tight loop

# Start rangefinder collection thread
rangefinder_thread = threading.Thread(target=collect_rangefinder, daemon=True)
rangefinder_thread.start()


def get_next_log_filename(prefix='log_', suffix='.csv'):
    log_files = [f for f in os.listdir('.') if re.match(rf'{prefix}\d{{3}}{suffix}', f)]
    max_number = -1
    for f in log_files:
        match = re.match(rf'{prefix}(\d{{3}}){suffix}', f)
        if match:
            num = int(match.group(1))
            if num > max_number:
                max_number = num
    return f"{prefix}{max_number + 1:03d}{suffix}"

# Initialize CSV file
try:
    filename = get_next_log_filename()
    print(f"Created: {filename}")
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'dwm_x_m', 'dwm_y_m', 'dwm_z_m', 'dwm_qf', 'drone_alt_m'])
except Exception as e:
    print(f"Failed to create CSV file: {e}")
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    sock.close()
    exit(1)
except KeyboardInterrupt:
    print("Interrupted by user (Ctrl+C)")
    print(f"Sending SIGINT to PID: {process.pid}")
    os.kill(process.pid, signal.SIGINT)
    sock.close()
    exit(1)

try:
    while True:
        start_time = time.time()

        # Receive UDP data (non-blocking)
        dwm_data = None
        try:
            data, addr = sock.recvfrom(1024)
            dwm_data = data.decode('utf-8').strip()
            # print(f"Received UDP: {dwm_data}")
        except socket.timeout:
            print("No UDP data received")

        # Parse DWM1001 data (format: ts:sec.usec [x,y,z,qf])
        dwm_x, dwm_y, dwm_z, dwm_qf, timestamp = None, None, None, None, None
        if dwm_data:
            try:
                # parts = dwm_data.split('[')[1].split(']')[0].split(',')
                parts = dwm_data.strip().split(',')
                timestamp = float(parts[0])
                dwm_x = float(parts[1]) / 1000.0  # mm to meters
                dwm_y = float(parts[2]) / 1000.0  # mm to meters
                dwm_z = float(parts[3]) / 1000.0  # mm to meters
                dwm_qf = int(parts[4])
            except Exception as e:
                print(f"Error parsing UDP data: {e}")

        # Get latest altitude
        with alt_lock:
            alt = latest_alt

        # Write to CSV
        try:
            with open(filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    f"{timestamp:.2f}" if timestamp else "",
                    f"{dwm_x:.3f}" if dwm_x else "",
                    f"{dwm_y:.3f}" if dwm_y else "",
                    f"{dwm_z:.3f}" if dwm_z else "",
                    f"{dwm_qf}" if dwm_qf else "",
                    f"{alt:.3f}" if alt else ""
                ])
        except Exception as e:
            print(f"Error writing to CSV: {e}")

        # Maintain strict 10Hz
        elapsed = time.time() - start_time
        sleep_time = max(0, TIME_PERIOD - elapsed)
        time.sleep(sleep_time)

except KeyboardInterrupt:
    print("Terminating...")
    print(f"Sending SIGINT to PID: {process.pid}")
    os.kill(process.pid, signal.SIGINT)
    time.sleep(0.5)  # Allow time for cleanup
    sock.close()