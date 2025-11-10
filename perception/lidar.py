import matplotlib.pyplot as plt
import numpy as np
import math
import time
import json
import os
import threading
from rplidar import RPLidar, RPLidarException
from pathlib import Path      

#current robot pose stored here and accessed via get_pose/update_pose()
_pose_lock = threading.Lock()
_current_pose = [0.0, 0.0, 0.0]

PORT = '/dev/ttyUSB0' #lidar's serial port
#port name on linux/macos = /dev/ttyUSB0
#port name on windows = COM3 or COM4

POSE_PATH = Path("/tmp/ic_pose.json")  

BAUDRATE = 115200

def _write_pose_file(x: float, y: float, th: float) -> None:
    """Atomically write the latest pose so other processes can read it."""
    import json, os, tempfile
    try:
        d = {"x": float(x), "y": float(y), "theta": float(th)}
        fd, tmp = tempfile.mkstemp(prefix="ic_pose_", dir="/tmp")
        with os.fdopen(fd, "w") as f:
            json.dump(d, f)
        os.replace(tmp, POSE_PATH)  # atomic rename
    except Exception:
        pass  # never crash the LiDAR thread

def _write_pose_json(x: float, y: float, theta: float) -> None:
    try:
        tmp = POSE_PATH.with_suffix(".tmp")
        tmp.write_text(json.dumps({"x": float(x), "y": float(y), "theta": float(theta)}))
        tmp.replace(POSE_PATH)
        os.chmod(POSE_PATH, 0o644)
    except Exception:
        pass

def update_pose(x, y, theta):
    """Update in-memory pose and publish to the shared file."""
    with _pose_lock:
        _current_pose[:] = [float(x), float(y), float(theta)]
    _write_pose_json(x, y, theta)           # single place that writes

# def get_pose():
#     """Return the most recent pose estimate (x, y, theta)."""
#     with _pose_lock:
#         return tuple(_current_pose)

def get_pose():
    """Return the most recent pose estimate (x, y, theta)."""
    try:
        if POSE_PATH.exists():
            data = json.loads(POSE_PATH.read_text())
            return (float(data.get("x", 0.0)),
                    float(data.get("y", 0.0)),
                    float(data.get("theta", 0.0)))
    except Exception:
        # file may be mid-write; ignore and fall back
        pass
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

            update_pose(x, y, theta)   # this now writes the file via _write_pose_json
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
