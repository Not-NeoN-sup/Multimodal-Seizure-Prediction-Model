import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import time
import threading
import queue
import pandas as pd

class MPU6500Plotter:
    def __init__(self, host='0.0.0.0', port=8888, max_points=100):
        self.host = host
        self.port = port
        self.max_points = max_points
        
        # Data storage
        self.data_queue = queue.Queue()
        self.timestamps = deque(maxlen=max_points)
        self.accel_x = deque(maxlen=max_points)
        self.accel_y = deque(maxlen=max_points)
        self.accel_z = deque(maxlen=max_points)
        self.gyro_x = deque(maxlen=max_points)
        self.gyro_y = deque(maxlen=max_points)
        self.gyro_z = deque(maxlen=max_points)
        self.roll = deque(maxlen=max_points)
        self.pitch = deque(maxlen=max_points)
        
        # Socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(1.0)  # 1 second timeout
        
        self.running = True
        self.start_time = time.time()
        
        # Start data receiving thread
        self.data_thread = threading.Thread(target=self.receive_data)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        # Setup plots
        self.setup_plots()
        
    def receive_data(self):
        """Receive data from UDP socket in separate thread"""
        print(f"Listening for data on {self.host}:{self.port}")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                data_str = data.decode('utf-8').strip()
                self.parse_data(data_str)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving data: {e}")
                
    def parse_data(self, data_str):
        """Parse incoming CSV data string"""
        try:
            parts = data_str.split(',')
            if len(parts) >= 9:
                timestamp = int(parts[0])  # Milliseconds
                ax = float(parts[1])
                ay = float(parts[2])
                az = float(parts[3])
                gx = float(parts[4])
                gy = float(parts[5])
                gz = float(parts[6])
                roll = float(parts[7])
                pitch = float(parts[8])
                
                # Convert timestamp to relative seconds
                rel_time = (timestamp - (self.start_time * 1000)) / 1000.0
                
                data_point = {
                    'timestamp': rel_time,
                    'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'roll': roll, 'pitch': pitch
                }
                
                self.data_queue.put(data_point)
                
        except (ValueError, IndexError) as e:
            print(f"Error parsing data: {e}")
            
    def setup_plots(self):
        """Setup matplotlib plots"""
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('MPU6500 Real-time Data', fontsize=16)
        
        # Accelerometer plot
        self.ax_accel = self.axes[0, 0]
        self.ax_accel.set_title('Accelerometer Data (g)')
        self.ax_accel.set_xlabel('Time (s)')
        self.ax_accel.set_ylabel('Acceleration (g)')
        self.line_ax, = self.ax_accel.plot([], [], 'r-', label='X', linewidth=2)
        self.line_ay, = self.ax_accel.plot([], [], 'g-', label='Y', linewidth=2)
        self.line_az, = self.ax_accel.plot([], [], 'b-', label='Z', linewidth=2)
        self.ax_accel.legend()
        self.ax_accel.grid(True, alpha=0.3)
        
        # Gyroscope plot
        self.ax_gyro = self.axes[0, 1]
        self.ax_gyro.set_title('Gyroscope Data (°/s)')
        self.ax_gyro.set_xlabel('Time (s)')
        self.ax_gyro.set_ylabel('Angular Velocity (°/s)')
        self.line_gx, = self.ax_gyro.plot([], [], 'r-', label='X', linewidth=2)
        self.line_gy, = self.ax_gyro.plot([], [], 'g-', label='Y', linewidth=2)
        self.line_gz, = self.ax_gyro.plot([], [], 'b-', label='Z', linewidth=2)
        self.ax_gyro.legend()
        self.ax_gyro.grid(True, alpha=0.3)
        
        # Roll and Pitch plot
        self.ax_attitude = self.axes[1, 0]
        self.ax_attitude.set_title('Attitude (Roll & Pitch)')
        self.ax_attitude.set_xlabel('Time (s)')
        self.ax_attitude.set_ylabel('Angle (degrees)')
        self.line_roll, = self.ax_attitude.plot([], [], 'orange', label='Roll', linewidth=2)
        self.line_pitch, = self.ax_attitude.plot([], [], 'purple', label='Pitch', linewidth=2)
        self.ax_attitude.legend()
        self.ax_attitude.grid(True, alpha=0.3)
        
        # 3D Accelerometer magnitude plot
        self.ax_mag = self.axes[1, 1]
        self.ax_mag.set_title('Acceleration Magnitude')
        self.ax_mag.set_xlabel('Time (s)')
        self.ax_mag.set_ylabel('|Acceleration| (g)')
        self.line_mag, = self.ax_mag.plot([], [], 'black', linewidth=2)
        self.ax_mag.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
    def update_plot(self, frame):
        """Animation update function"""
        # Process queued data
        while not self.data_queue.empty():
            try:
                data_point = self.data_queue.get_nowait()
                
                self.timestamps.append(data_point['timestamp'])
                self.accel_x.append(data_point['ax'])
                self.accel_y.append(data_point['ay'])
                self.accel_z.append(data_point['az'])
                self.gyro_x.append(data_point['gx'])
                self.gyro_y.append(data_point['gy'])
                self.gyro_z.append(data_point['gz'])
                self.roll.append(data_point['roll'])
                self.pitch.append(data_point['pitch'])
                
            except queue.Empty:
                break
        
        if len(self.timestamps) > 0:
            # Update accelerometer plot
            self.line_ax.set_data(self.timestamps, self.accel_x)
            self.line_ay.set_data(self.timestamps, self.accel_y)
            self.line_az.set_data(self.timestamps, self.accel_z)
            
            # Update gyroscope plot
            self.line_gx.set_data(self.timestamps, self.gyro_x)
            self.line_gy.set_data(self.timestamps, self.gyro_y)
            self.line_gz.set_data(self.timestamps, self.gyro_z)
            
            # Update attitude plot
            self.line_roll.set_data(self.timestamps, self.roll)
            self.line_pitch.set_data(self.timestamps, self.pitch)
            
            # Update magnitude plot
            mag = [np.sqrt(ax**2 + ay**2 + az**2) for ax, ay, az in 
                   zip(self.accel_x, self.accel_y, self.accel_z)]
            self.line_mag.set_data(self.timestamps, mag)
            
            # Rescale plots
            for ax in [self.ax_accel, self.ax_gyro, self.ax_attitude, self.ax_mag]:
                ax.relim()
                ax.autoscale_view()
        
        return (self.line_ax, self.line_ay, self.line_az, 
                self.line_gx, self.line_gy, self.line_gz,
                self.line_roll, self.line_pitch, self.line_mag)
    
    def start_plotting(self):
        """Start the real-time plotting"""
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping plotter...")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the plotter and clean up"""
        self.running = False
        self.sock.close()
        
    def save_data_to_csv(self, filename='mpu6500_received_data.csv'):
        """Save current data to CSV file"""
        if len(self.timestamps) > 0:
            df = pd.DataFrame({
                'timestamp': list(self.timestamps),
                'accel_x': list(self.accel_x),
                'accel_y': list(self.accel_y),
                'accel_z': list(self.accel_z),
                'gyro_x': list(self.gyro_x),
                'gyro_y': list(self.gyro_y),
                'gyro_z': list(self.gyro_z),
                'roll': list(self.roll),
                'pitch': list(self.pitch)
            })
            df.to_csv(filename, index=False)
            print(f"Data saved to {filename}")

if __name__ == "__main__":
    print("MPU6500 Real-time Data Plotter")
    print("Make sure your C++ program is sending data to this IP address")
    print("Press Ctrl+C to stop")
    
    # Create and start plotter
    plotter = MPU6500Plotter(host='0.0.0.0', port=8888, max_points=200)
    
    try:
        plotter.start_plotting()
    except KeyboardInterrupt:
        print("\nShutting down...")
        plotter.save_data_to_csv()
    finally:
        plotter.stop()