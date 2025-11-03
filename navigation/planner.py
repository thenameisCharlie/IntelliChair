"""
planner.py - Navigation: Places + Simple Planner
- Safe (optional) plotting
- Abort-on-stop
- No GPIO cleanup between goals
- No simulated progress: declares arrival only with real pose feedback
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
THRESH_CM     = float(TUNABLES.get("THRESH_CM", 40))        # arrival threshold (cm)
CRUISE_SPEED  = int(TUNABLES.get("CRUISE_SPEED", 60))       # motor speed (0-100)
SLOW_SPEED    = int(TUNABLES.get("SLOW_SPEED", 30))         # slower speed near goal
STOP_DIST_CM  = float(TUNABLES.get("STOP_DIST_CM", 15))     # emergency stop distance
SPIN_SPEED    = int(TUNABLES.get("SPIN_SPEED", 50))         # spin speed for rotate
LOOP_DT       = float(TUNABLES.get("LOOP_DT", 0.05))        # loop tick seconds
TIME_PER_M    = float(TUNABLES.get("TIME_PER_M", 6.0))      # open-loop secs per meter (fallback)

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
        """Crude time-based spinner toward target heading (left/right by sign)."""
        deg = math.degrees(target_theta)
        print(f"[planner] Rotating toward θ={deg:.1f}°")
        duration = min(abs(target_theta), math.radians(90))
        try:
            if deg > 0:
                self.motors.spin_left(SPIN_SPEED)
            else:
                self.motors.spin_right(SPIN_SPEED)
            time.sleep(duration)
        finally:
            self.motors.stop()

    # --- Pose access (safe wrapper) ---
    def _read_pose(self):
        try:
            from perception.lidar import get_pose as lidar_get_pose
        except Exception:
            lidar_get_pose = None

        if not lidar_get_pose:
            return None
        try:
            p = lidar_get_pose()
            if p is None:
                return None
            if isinstance(p, (list, tuple)) and len(p) >= 3:
                return Pose(float(p[0]), float(p[1]), float(p[2]))
            if isinstance(p, dict):
                return Pose(float(p.get("x", 0.0)),
                            float(p.get("y", 0.0)),
                            float(p.get("theta", 0.0)))
            if hasattr(p, "x") and hasattr(p, "y") and hasattr(p, "theta"):
                return Pose(float(p.x), float(p.y), float(p.theta))
        except Exception:
            return None
        return None

    def _forward_until(self, distance_m: float, heading: float, start_pose: Pose, goal_pose: Pose) -> bool:
        """
        Drive forward toward goal by distance_m (meters).
        Returns True only if real pose says we're within THRESH_CM of goal.
        If no SLAM pose is available, performs open-loop motion for a short time and returns False.
        """
        print(f"[planner] Forwarding {distance_m:.2f} m toward heading {math.degrees(heading):.1f}°")

        have_pose = self._read_pose() is not None
        start_time = time.time()
        max_open_loop = TIME_PER_M * max(0.2, float(distance_m))  # minimum motion burst

        while True:
            # ABORT on stop
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

            # Slow near goal only if we have pose feedback to judge nearness
            speed = SLOW_SPEED if have_pose else CRUISE_SPEED
            try:
                self.motors.forward(int(speed))
            except Exception:
                try:
                    self.motors.go_forward(int(speed))
                except Exception:
                    pass  # tolerate different motor APIs

            time.sleep(LOOP_DT)

            # With SLAM pose: recompute remaining from REAL pose and decide arrival
            if have_pose:
                pnow = self._read_pose()
                if pnow is not None:
                    remaining_m = self._distance_to_goal(pnow, goal_pose)
                    # Optional: print every ~0.5s to avoid spam
                    if int((time.time() - start_time) * 10) % 5 == 0:
                        print(f"[planner] Remaining {remaining_m:.2f} m (pose-based)")
                    if remaining_m <= (THRESH_CM / 100.0):
                        self.motors.stop()
                        return True
                continue

            # No pose (open-loop): run for time budget then stop—do NOT claim arrival
            if (time.time() - start_time) >= max_open_loop:
                self.motors.stop()
                print("[planner] Open-loop motion complete (no pose available).")
                return False

    # --- Pose source fallback ---
    def _get_current_pose(self) -> Pose:
        p = self._read_pose()
        if p is not None:
            return p
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
            arrived = self._forward_until(distance, heading, current_pose, goal_pose)
        except Exception as e:
            print(f"[planner] Error while navigating: {e}")
            arrived = False
        finally:
            # Always stop motors between phases; keep GPIO alive for future commands
            try:
                self.motors.stop()
            except Exception:
                pass

        if arrived:
            print(f"[planner] Arrived at '{name}' (within tolerance).")
        else:
            print(f"[planner] Could not reach '{name}' due to obstacle or error.")

        # Optional plotting (ignored if matplotlib not available)
        if PLOT:
            try:
                plt.figure()
                plt.title(f"Path to {name}")
                plt.plot([current_pose.x, goal_pose.x], [current_pose.y, goal_pose.y], 'ro-')
                plt.savefig(f"planner_path_{name}.png")
            except Exception:
                pass
            finally:
                try:
                    plt.close()
                except Exception:
                    pass
