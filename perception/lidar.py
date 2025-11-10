import matplotlib.pyplot as plt
import numpy as np
import math
import time
import json
import os
import threading
from rplidar import RPLidar, RPLidarException

#current robot pose stored here and accessed via get_pose/update_pose()
_pose_lock = threading.Lock()
_current_pose = [0.0, 0.0, 0.0]

PORT = '/dev/ttyUSB0' #lidar's serial port
#port name on linux/macos = /dev/ttyUSB0
#port name on windows = COM3 or COM4

POSE_PATH = "/tmp/ic_pose.json"

BAUDRATE = 115200

def _write_pose_file(x, y, theta, path=POSE_PATH):
    try:
        import json, time
        with open(path, "w") as f:
            json.dump({"x": x, "y": y, "theta": theta, "ts": time.time()}, f)
    except Exception:
        pass

def update_pose(x, y, theta):
    """Update the estimated robot pose."""
    with _pose_lock:
        _current_pose[0] = x
        _current_pose[1] = y
        _current_pose[2] = theta

def get_pose():
    """Return the most recent pose estimate (x, y, theta)."""
    with _pose_lock:
        return tuple(_current_pose)

def lidar_thread(port=PORT, baudrate=BAUDRATE):
    """Continuously read lidar scans and update robot pose."""
    try:
        lidar = RPLidar(port, baudrate)
        print("[lidar] Connected & scanning...")
        total_rotation = 0.0

        for scan in lidar.iter_scans():  # scan is a list of (quality, angle, distance)
            if not scan:
                continue
            # Compute average distance
            distances = [d for q, a, d in scan if d > 0]
            avg_dist = sum(distances)/len(distances) if distances else 0.0

            # Simple pose simulation
            total_rotation += 0.01  # incremental rotation
            x = avg_dist * math.cos(total_rotation) / 2000.0
            y = avg_dist * math.sin(total_rotation) / 2000.0
            theta = total_rotation

            update_pose(x, y, theta)
            _write_pose_file(x, y, theta) 
            time.sleep(0.05)

    except RPLidarException as e:
        print(f"[lidar] RPLidar error: {e}")
    except Exception as e:
        print(f"[lidar] Error: {e}")
    finally:
        try:
            lidar.stop()
            lidar.disconnect()
        except:
            pass
        print("[lidar] Stopped.")

def run():
    lidar = RPLidar(PORT) #object to communicate with the sensor
    print("Starting RPLidar...")
    
   
    fig, ax = plt.subplots() #visualizes data into cartesian plot
    scatter = ax.scatter([], [], c=[], cmap = 'jet', s=5,
    vmin = 0, vmax = 4000) #empty scatter plot
    robot_dot, = ax.plot(0,0, 'ro', markersize = 8, label = "Robot")
    ax.set_xlim(-4000, 4000)#sets coordinate limits
    ax.set_ylim(-4000, 4000)#sets coordinate limits 
    ax.set_aspect('equal')
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.legend(loc = "upper right")
    plt.ion() #interactive mode for live updates
    plt.show()

    try:
        while True:
            try:
                #makes for continuous 360 degree scans at around 5-10 Hz
                #5-10 hz means 5 to 10 full 360 degree scans per second
                for scan in lidar.iter_scans():
                    xs, ys, cs = [], [], []
                    for (_, angle, distance) in scan:
                        rad = math.radians(angle) #converts angle(degrees) into radian
                        x = distance * math.cos(rad) #computes x
                        y = distance * math.sin(rad) #computes y
                        xs.append(x)
                        ys.append(y)
                        cs.append(distance) #shows colors by distance
                    if xs and ys:
                        scatter.set_offsets(np.c_[xs, ys]) #updates scatter plot with fresh data
                        scatter.set_array(np.array(cs))
                        plt.draw()
                        plt.pause(0.01) #plot refreshing
            except RPLidarException as e:
                print("RPLidar error:",e, "-> restarting scan...")
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
                time.sleep(1) #gives the lidar a moment
                lidar = RPLidar(PORT) #reconnects after the error
    
    except KeyboardInterrupt:
        print('Stopping...') #pressing Ctrl+C
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        plt.ioff()
        plt.show()
if __name__ == '__main__': #differentiates main program from imported module
    #run()
    t = threading.Thread(target=lidar_thread, daemon=True)
    t.start()

    try:
        while True:
            print("[pose]", get_pose())
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[lidar] Exiting...")
