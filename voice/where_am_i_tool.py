from __future__ import annotations
import rclpy
from rclpy.node import Node 
from std_srvs.srv import Trigger
import threading, math, time
from rplidar import RPLidar, RPLidarException
#from pathlib import Path
#import subprocess

PORT = '/dev/ttyUSB0' #port for lidar sensor 
BAUDRATE = 115200 #baud rate

_pose_lock = threading.Lock()
_current_pose = [0.0, 0.0, 0.0]  #saving robot's location in a variable

def update_pose(x, y, theta): #to update the robot's 
    with _pose_lock:
        _current_pose[0], _current_pose[1], _current_pose[2] = x, y, theta

def get_pose(): #recent robot's location
    with _pose_lock:
        return tuple(_current_pose)


def lidar_thread(): #to read the scans continuously and update locaton
    try:
        lidar = RPLidar(PORT, BAUDRATE)
        print("[lidar] Connected and scanning...")
        total_rotation = 0.0 

        #read scans in a loop
        for scan in lidar.iter_scans():
            if not scan:
                continue

            distances = [d for q, a, d in scan if d > 0]
            avg_dist = sum(distances) / len(distances) if distances else 0.0

            #sample movement pattern
            total_rotation += 0.01
            x = avg_dist * math.cos(total_rotation) / 2000.0
            y = avg_dist * math.sin(total_rotation) / 2000.0
            theta = total_rotation

            #saves location
            update_pose(x,y,theta)
            time.sleep(0.05)

    
    except (RPLidarException, Exception) as e:
        print(f"[lidar] Error: {e}")

    finally: #stop and disconnect
        try:
            lidar.stop()
            lidar.disconnect()
        except Exception:
            pass
        print("[lidar] Stopped.")


#boundaries for the rooms
def coordinates_to_room(x, y):
    if -1 < x < 2 and -1 < y < 3:
        return "Living Room"
    
    elif 3 < x < 6 and -1 < y < 2:
        return "Kitchen"

    elif -2 < x < 1 and 4 < y < 7:
        return "Bedroom"

    else:
        return "Unknown area"

#ros2 node
class WhereAmINode(Node):
    def __init__(self):
        #trigger service called where am i created
        super().__init__('Where_am_i_node')
        self.create_service(Trigger, 'where_am_i', self.handle_where_am_i)
        self.get_logger().info("where_am_i service ready.")

    def handle_where_am_i(self, request, response):
        #handles requests
        x, y, theta = get_pose()
        room = coordinates_to_room(x, y)
        response.success = True
        response.message = (f"You are in the {room} at "
            f"({x:.2f}, {y:.2f}), facing {math.degrees(theta):.1f}°")
        return response


#main 
def main():
    #start lidar thread and spin ros2 node
    t = threading.Thread(target = lidar_thread, daemon = True)
    t.start()

    #start ros2 node
    rclpy.init()
    node = WhereAmINode()

    try: 
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("[main] Shutdown complete.")


if __name__ == "__main__":
    main()
