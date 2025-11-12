from __future__ import annotations
import math, json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

#try importing the SLAM pose function
try:
    from perception.slam import _get_current_pose_from_lidar
except Exception:
    _get_current_pose_from_lidar = None

# file that lidar/slam writes the last known pose to
POSE_PATH = Path("/tmp/ic_pose.json")

#ROOM LOGIC
def coordinates_to_room(x: float, y: float) -> str:
    """Determine which room the robot is in based on coordinate ranges."""
    if -1 < x < 2 and -1 < y < 3:
        return "Living Room"
    elif 3 < x < 6 and -1 < y < 2:
        return "Kitchen"
    elif -2 < x < 1 and 4 < y < 7:
        return "Bedroom"
    else:
        return "Unknown area"


#POSE HELPERS
def get_pose_with_fallback() -> tuple[float, float, float]:
    """Try SLAM pose first; fall back to /tmp/ic_pose.json."""
    # 1st Try the live SLAM function if available
    if _get_current_pose_from_lidar is not None:
        try:
            x, y, theta = _get_current_pose_from_lidar()
            # If valid numeric values
            if any(abs(v) > 1e-6 for v in (x, y, theta)):
                return x, y, theta
        except Exception:
            pass

    # 2️nd Fallback: read JSON file if it exists
    try:
        if POSE_PATH.exists():
            data = json.loads(POSE_PATH.read_text())
            return (
                float(data.get("x", 0.0)),
                float(data.get("y", 0.0)),
                float(data.get("theta", 0.0)),
            )
    except Exception:
        pass

    # 3rd Final fallback
    return 0.0, 0.0, 0.0


#ROS2 NODE
class WhereAmINode(Node):
    """ROS2 node exposing a Trigger service called 'where_am_i'."""

    def __init__(self):
        super().__init__('where_am_i_node')
        self.create_service(Trigger, 'where_am_i', self.handle_where_am_i)
        self.get_logger().info(" where_am_i service ready (with fallback).")

    def handle_where_am_i(self, request, response):
        x, y, theta = get_pose_with_fallback()
        room = coordinates_to_room(x, y)

        response.success = True
        response.message = (
            f"You are in the {room} at "
            f"({x:.2f}, {y:.2f}), facing {math.degrees(theta):.1f}°."
        )

        self.get_logger().info(response.message)
        return response


#MAIN
def main():
    rclpy.init()
    node = WhereAmINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[main] Shutdown complete.")


if __name__ == "__main__":
    main()

