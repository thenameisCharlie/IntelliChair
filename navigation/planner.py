"""
planner.py - Complete Navigation System Test
Tests integration of SLAM, Places, and Path Planning
"""

import math
import time
import threading
from typing import Optional

from hardware.motors import YahboomMotors
from hardware.ultrasonic import distance_filtered_cm
from navigation.places import PlaceManager, Pose
from utils.config import TUNABLES

# --- Safe plotting setup (optional, won't crash if matplotlib missing) ---
try:
    import matplotlib.pyplot as plt
    PLOT = True
except Exception:
    PLOT = False
    class _NoPlot:
        def __getattr__(self, name):
            return lambda *a, **k: None
    plt = _NoPlot()

# Tunables / thresholds (with sane fallbacks)
THRESH_CM     = TUNABLES.get("THRESH_CM", 40)          # arrival threshold (cm)
CRUISE_SPEED  = TUNABLES.get("CRUISE_SPEED", 60)       # motor speed (0-100)
SLOW_SPEED    = TUNABLES.get("SLOW_SPEED", 30)         # slower speed near goal
STOP_DIST_CM  = TUNABLES.get("STOP_DIST_CM", 15)       # emergency stop distance
SPIN_SPEED    = int(TUNABLES.get("SPIN_SPEED", 50))    # spin speed for rotate
LOOP_DT       = float(TUNABLES.get("LOOP_DT", 0.05))   # loop tick seconds
SPEED_TO_MPS  = float(TUNABLES.get("SPEED_TO_MPS", 0.05))  # rough conversion m/s per speed%

class SimplePlanner:
    def __init__(self):
        self.pm = PlaceManager()
        self.motors = YahboomMotors()
        self.abort = threading.Event()   # set() to abort, clear() to resume

    # --- Geometry helpers ---
    def _heading_to_goal(self, current: Pose, goal: Pose) -> float:
        return math.atan2(goal.y - current.y, goal.x - current.x)

    def _distance_to_goal(self, current: Pose, goal: Pose) -> float:
        return math.hypot(goal.x - current.x, goal.y - current.y)

    # --- Motion primitives ---
    def _rotate_to_heading(self, target_theta: float):
        """Crude time-based spinner toward target heading."""
        deg = math.degrees(target_theta)
        print(f"[planner] Rotating toward θ={deg:.1f}°")
        duration = min(abs(target_theta), math.radians(90))
        if deg > 0:
            self.motors.spin_left(SPIN_SPEED)
        else:
            self.motors.spin_right(SPIN_SPEED)
        time.sleep(duration)
        self.motors.stop()

    def _forward_until(self, distance_m: float, heading: float, current: Pose) -> bool:
        """
        Drive forward toward goal by distance_m (meters).
        Returns True if arrived within tolerance, False if aborted/blocked.
        """
        print(f"[planner] Forwarding {distance_m:.2f} m toward heading {math.degrees(heading):.1f}°")

        remaining = max(0.0, float(distance_m))
        speed = CRUISE_SPEED
        last_time = time.time()

        try:
            from perception import lidar as lidar_module  # optional
        except Exception:
            lidar_module = None

        while remaining > (THRESH_CM / 100.0):
            # ABORT if stop command happened
            if self.abort.is_set():
                print("[planner] ABORT: stop command received.")
                self.motors.stop()
                self.abort.clear()
                return False

            # Obstacle check
            d = distance_filtered_cm()
            if d is not None and d <= STOP_DIST_CM:
                print(f"[planner] Obstacle at {d:.0f} cm → stopping.")
                self.motors.stop()
                return False

            # Slow near goal
            if remaining < 0.5:
                speed = SLOW_SPEED

            # Drive forward
            try:
                self.motors.forward(int(speed))
            except Exception:
                try:
                    self.motors.go_forward(int(speed))
                except Exception:
                    pass  # best-effort across implementations

            # Dead-reckon progress (very rough)
            now = time.time()
            dt = now - last_time
            last_time = now
            traveled = SPEED_TO_MPS * speed * dt
            dx = math.cos(heading) * traveled
            dy = math.sin(heading) * traveled
            new_x = current.x + dx
            new_y = current.y + dy
            new_theta = heading

            if lidar_module and hasattr(lidar_module, "update_pose"):
                try:
                    lidar_module.update_pose(new_x, new_y, new_theta)
                except Exception:
                    pass

            current = Pose(new_x, new_y, new_theta)
            # Update remaining (straight-line approximation)
            remaining = max(0.0, remaining - traveled)

            time.sleep(LOOP_DT)

        self.motors.stop()
        return True

    # --- Pose source ---
    def _get_current_pose(self) -> Pose:
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
                    return Pose(float(p.get("x", 0.0)),
                                float(p.get("y", 0.0)),
                                float(p.get("theta", 0.0)))
                if hasattr(p, "x") and hasattr(p, "y") and hasattr(p, "theta"):
                    return Pose(float(p.x), float(p.y), float(p.theta))
            except Exception as e:
                print(f"[planner] Warning: error reading pose: {e}")
        return Pose(0.0, 0.0, 0.0)

    # --- Public API ---
    def set_goal(self, name: str, current_pose: Optional[Pose] = None):
        goal = self.pm.get_place(name)
        if not goal:
            print(f"[planner] Unknown destination: {name}")
            return

        goal_pose = Pose(float(goal["x"]), float(goal["y"]), float(goal["theta"]))
        print(f"[planner] Destination '{name}' → ({goal_pose.x:.3f}, {goal_pose.y:.3f}, θ={goal_pose.theta:.3f})")

        if current_pose is None:
            current_pose = self._get_current_pose()
            print(f"[planner] Current pose → ({current_pose.x:.3f}, {current_pose.y:.3f}, θ={current_pose.theta:.3f})")

        heading = self._heading_to_goal(current_pose, goal_pose)
        distance = self._distance_to_goal(current_pose, goal_pose)
        print(f"[planner] Path: heading={math.degrees(heading):.1f}°, distance={distance:.2f} m")

        # ensure no stale abort before starting
        self.abort.clear()

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

        # Stop motors (keep GPIO active so future commands work)
        try:
            self.motors.stop()
        except Exception:
            pass

        # Optional plotting (ignored if matplotlib not available)
        if PLOT:
            plt.figure()
            plt.title(f"Path to {name}")
            try:
                plt.plot([current_pose.x, goal_pose.x], [current_pose.y, goal_pose.y], 'ro-')
                plt.savefig(f"planner_path_{name}.png")
            except Exception:
                pass
            finally:
                plt.close()
