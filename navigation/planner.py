"""
planner.py - Complete Navigation System Test
Tests integration of SLAM, Places, and Path Planning
"""

#update import(Frank)
import math
import time
from typing import Optional

from hardware.motors import YahboomMotors
from hardware.ultrasonic import distance_filtered_cm
from navigation.places import PlaceManager, Pose
from utils.config import TUNABLES

#thresh for distance
THRESH_CM = TUNABLES.get("THRESH_CM", 40)        # arrival threshold in cm
CRUISE_SPEED = TUNABLES.get("CRUISE_SPEED", 60) # motor speed for cruise (0-100)
SLOW_SPEED = TUNABLES.get("SLOW_SPEED", 30)     # slower speed near goal
STOP_DIST_CM = TUNABLES.get("STOP_DIST_CM", 15)  # emergency stop (cm)

class SimplePlanner:
    def __init__(self):
        self.pm = PlaceManager()
        self.motors = YahboomMotors()

    def _heading_to_goal(self, current: Pose, goal: Pose) -> float:
        return math.atan2(goal.y - current.y, goal.x - current.x)

    def _distance_to_goal(self, current: Pose, goal: Pose) -> float:
        return math.hypot(goal.x - current.x, goal.y - current.y)

    def _rotate_to_heading(self, target_theta: float):
        """Spin robot toward desired heading (crude time-based spinner)."""
        deg = math.degrees(target_theta)
        print(f"[planner] Rotating toward θ={deg:.1f}°")
        #rotation
        duration = min(abs(target_theta), math.radians(90))
        if deg > 0:
            self.motors.spin_left(int(TUNABLES.get("SPIN_SPEED", 50)))
        else:
            self.motors.spin_right(int(TUNABLES.get("SPIN_SPEED", 50)))
        time.sleep(duration)
        self.motors.stop()

    def _forward_until(self, distance_m: float, heading: float, current: Pose) -> bool:
        """
        Drive forward toward goal by distance_m (meters). Returns True if arrived,
        False if stopped by obstacle.
        """
        print(f"[planner] Forwarding {distance_m:.2f} m toward heading {math.degrees(heading):.1f}°")

        #convert meters to meters
        remaining = distance_m
        speed = CRUISE_SPEED
        last_time = time.time()

        from perception import lidar as lidar_module

        #loop until close enough or obstacle
        while remaining > (THRESH_CM / 100.0):
            #obstacle check
            d = distance_filtered_cm()
            if d is not None and d <= STOP_DIST_CM:
                print(f"[planner] Obstacle detected at {d} cm → stopping.")
                self.motors.stop()
                return False

            #choose speed slow down when close
            if remaining < 0.5:
                speed = SLOW_SPEED

            #command motors forward
            try:
                self.motors.forward(int(speed))
            except Exception:
                #fallback to approximate movement
                try:
                    self.motors.go_forward(int(speed))
                except Exception:
                    pass

            #update pose approximation using simple integration
            now = time.time()
            dt = now - last_time
            last_time = now
            #approximate forward distance traveled since last loop (m)
            conversion_factor = TUNABLES.get("SPEED_TO_MPS", 0.05)
            traveled = conversion_factor * speed * dt
            dx = math.cos(heading) * traveled
            dy = math.sin(heading) * traveled
            new_x = current.x + dx
            new_y = current.y + dy
            new_theta = heading
            #update perception.lidar pose so SLAM/teach can read it
            try:
                lidar_module.update_pose(new_x, new_y, new_theta)
            except Exception:
                pass
            #refresh current and remaining distance
            current = Pose(new_x, new_y, new_theta)
            remaining = self._distance_to_goal(current, Pose(current.x + math.cos(heading)*remaining,
                                                              current.y + math.sin(heading)*remaining,
                                                              0))
            #recompute distance to goal more simply:
            #we had initial goal computed outside; better to compute from pm again or pass goal coords
            #but to keep this self-contained, break when remaining small
            if remaining <= (THRESH_CM / 100.0):
                break

            time.sleep(0.05)

        self.motors.stop()
        return True

    def _get_current_pose(self) -> Pose:
        """
        Try to obtain current SLAM pose from perception.lidar.get_pose()
        Returns Pose fallbacking to zeros if unavailable.
        """
        try:
            from perception.lidar import get_pose as lidar_get_pose
        except Exception:
            lidar_get_pose = None

        if lidar_get_pose:
            try:
                p = lidar_get_pose()
                if p is None:
                    return Pose(0.0, 0.0, 0.0)
                if isinstance(p, (list, tuple)) and len(p) >= 3:
                    return Pose(float(p[0]), float(p[1]), float(p[2]))
                if isinstance(p, dict):
                    return Pose(float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("theta", 0.0)))
                if hasattr(p, "x") and hasattr(p, "y") and hasattr(p, "theta"):
                    return Pose(float(p.x), float(p.y), float(p.theta))
            except Exception as e:
                print(f"[planner] Warning: error reading pose from perception.lidar: {e}")

        #fallback
        return Pose(0.0, 0.0, 0.0)

    def set_goal(self, name: str, current_pose: Optional[Pose] = None):
        """
        Navigate to saved place by name. If current_pose is not provided, attempt to read it.
        """
        goal = self.pm.get_place(name)
        if not goal:
            print(f"[planner] Unknown destination: {name}")
            return

        goal_pose = Pose(float(goal["x"]), float(goal["y"]), float(goal["theta"]))
        print(f"[planner] Destination '{name}' → ({goal_pose.x:.3f}, {goal_pose.y:.3f}, θ={goal_pose.theta:.3f})")

        if current_pose is None:
            current_pose = self._get_current_pose()
            print(f"[planner] Current pose (from lidar/SLAM) → ({current_pose.x:.3f}, {current_pose.y:.3f}, θ={current_pose.theta:.3f})")

        heading = self._heading_to_goal(current_pose, goal_pose)
        distance = self._distance_to_goal(current_pose, goal_pose)
        print(f"[planner] Path: heading={math.degrees(heading):.1f}°, distance={distance:.2f} m")

        #rotate then forward toward goal
        try:
            self._rotate_to_heading(heading)
            arrived = self._forward_until(distance, heading, current_pose)
        except Exception as e:
            print(f"[planner] Error while navigating: {e}")
            arrived = False

        if arrived:
            print(f"[planner] Arrived at '{name}' (within tolerance).")
        else:
            print(f"[planner] Could not reach '{name}' due to obstacle or error.")

        #stop and shutdown motors cleanly
        try:
            self.motors.stop()
            self.motors.shutdown()
        except Exception:
            pass
