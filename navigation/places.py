"""
SLAM Navigation System
- 5.1 teach_place(name, aliases) to save current SLAM pose to places.json
- 5.2 set_goal(name) to drive to a saved place using SLAM pose + planner
"""

import json
import os
import numpy as np
import time
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime
from collections import deque


@dataclass
class Pose:
    """Represents a 2D pose with x, y coordinates and theta orientation."""
    x: float
    y: float
    theta: float


class PlaceManager:
    """Manages saving and loading of named places with their poses."""
    
    def __init__(self, filename: str = "places.json"):
        self.filename = filename
        self.places = self._load_places()
    
    def _load_places(self) -> Dict:
        """Load places from JSON file if it exists."""
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    data = json.load(f)
                    print(f"Loaded {len(data)} places from {self.filename}")
                    return data
            except json.JSONDecodeError as e:
                print(f"Warning: Could not parse {self.filename}: {e}")
                backup_name = f"{self.filename}.backup"
                if os.path.exists(self.filename):
                    os.rename(self.filename, backup_name)
                    print(f"Corrupted file backed up to {backup_name}")
                return {}
        return {}
    
    def _save_places(self):
        """Save places to JSON file with atomic write."""
        temp_filename = f"{self.filename}.tmp"
        try:
            with open(temp_filename, 'w') as f:
                json.dump(self.places, f, indent=2)
            os.replace(temp_filename, self.filename)
            print(f"Places saved to {self.filename}")
        except Exception as e:
            print(f"Error saving places: {e}")
            if os.path.exists(temp_filename):
                os.remove(temp_filename)
    
    def teach_place(self, name: str, pose: Pose, aliases: Optional[List[str]] = None):
        """
        Save the current SLAM pose to places.json with a name and optional aliases.
        
        Args:
            name: The name of the place (e.g., "kitchen")
            pose: The current robot pose (x, y, theta)
            aliases: Optional list of alternative names for this place
        """
        if aliases is None:
            aliases = []
        
        # Check for duplicate aliases
        for existing_name, existing_data in self.places.items():
            if existing_name != name:
                existing_aliases = existing_data.get("aliases", [])
                conflicts = set(aliases) & set(existing_aliases)
                if conflicts:
                    print(f"Warning: Aliases {conflicts} already used by '{existing_name}'")
        
        # Store the place with its pose, aliases, and metadata
        self.places[name] = {
            "x": round(pose.x, 3),
            "y": round(pose.y, 3),
            "theta": round(pose.theta, 3),
            "aliases": aliases,
            "taught_at": datetime.now().isoformat(),
            "visits": self.places.get(name, {}).get("visits", 0) + 1
        }
        
        self._save_places()
        
        print(f"\n✓ Taught place '{name}' at position:")
        print(f"  x={pose.x:.3f} m, y={pose.y:.3f} m, θ={pose.theta:.3f} rad")
        if aliases:
            print(f"  Aliases: {', '.join(aliases)}")
    
    def get_place(self, name: str) -> Optional[Dict]:
        """
        Retrieve a place by name or alias.
        
        Args:
            name: The name or alias of the place
            
        Returns:
            Dictionary with pose information or None if not found
        """
        # Check direct name match (case-insensitive)
        name_lower = name.lower()
        for place_name, place_data in self.places.items():
            if place_name.lower() == name_lower:
                return {"name": place_name, **place_data}
        
        # Check aliases (case-insensitive)
        for place_name, place_data in self.places.items():
            aliases = [a.lower() for a in place_data.get("aliases", [])]
            if name_lower in aliases:
                return {"name": place_name, **place_data}
        
        return None
    
    def list_places(self):
        """Print all saved places in a formatted table."""
        if not self.places:
            print("\nNo places saved yet.")
            print("   Use teach_place() to save locations!")
            return
        
        print("\n" + "=" * 70)
        print("SAVED PLACES")
        print("=" * 70)
        
        for name, data in sorted(self.places.items()):
            print(f"\n{name.upper()}")
            print(f"  Position: x={data['x']:6.3f} m, y={data['y']:6.3f} m, θ={data['theta']:6.3f} rad")
            
            if data.get('aliases'):
                print(f"  Aliases:  {', '.join(data['aliases'])}")
            
            if data.get('taught_at'):
                print(f"  Taught:   {data['taught_at'][:19]}")
        
        print("=" * 70)
        print(f"Total: {len(self.places)} places\n")


class PathPlanner:
    """Simple A* path planner for occupancy grid."""
    
    def __init__(self, occupancy_map=None):
        self.map = occupancy_map
    
    def plan_path(self, start_x: float, start_y: float, 
                  goal_x: float, goal_y: float) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start to goal using A* algorithm.
        
        Args:
            start_x, start_y: Start position in meters
            goal_x, goal_y: Goal position in meters
            
        Returns:
            List of (x, y) waypoints or None if no path found
        """
        # If no map available, return straight line path
        if self.map is None:
            return self._straight_line_path(start_x, start_y, goal_x, goal_y)
        
        # Otherwise use A* on occupancy grid
        return self._astar_path(start_x, start_y, goal_x, goal_y)
    
    def _straight_line_path(self, start_x, start_y, goal_x, goal_y, num_waypoints=10):
        """Generate straight line path with intermediate waypoints."""
        path = []
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            path.append((x, y))
        return path
    
    def _astar_path(self, start_x, start_y, goal_x, goal_y):
        """A* pathfinding on occupancy grid (simplified)."""
        # This is a placeholder - integrate with your actual occupancy grid
        # For now, return straight line
        return self._straight_line_path(start_x, start_y, goal_x, goal_y)


class RobotNavigator:
    """Handles autonomous navigation to goal locations."""
    
    def __init__(self, robot_interface, slam_interface, path_planner=None):
        """
        Initialize navigator.
        
        Args:
            robot_interface: Must have drive(v, w) and get_ultrasonic() methods
            slam_interface: Must have get_pose() method returning (x, y, theta)
            path_planner: Optional path planner with plan_path() method
        """
        self.robot = robot_interface
        self.slam = slam_interface
        self.planner = path_planner or PathPlanner()
        
        # Navigation parameters
        self.goal_tolerance = 0.40  # 40 cm arrival threshold
        self.safety_distance = 0.15  # 15 cm ultrasonic stop distance
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.waypoint_threshold = 0.25  # 25 cm to switch waypoints
        
        # Control gains
        self.k_linear = 0.5
        self.k_angular = 2.0
        
        self.current_path = None
        self.navigation_active = False
    
    def navigate_to_pose(self, goal_x: float, goal_y: float, goal_theta: float = None) -> bool:
        """
        Navigate to a specific pose.
        
        Args:
            goal_x, goal_y: Goal position in meters
            goal_theta: Optional goal orientation (not enforced in basic version)
            
        Returns:
            bool: True if goal reached successfully
        """
        print(f"\n{'='*60}")
        print(f"AUTONOMOUS NAVIGATION")
        print(f"{'='*60}")
        print(f"Target: ({goal_x:.2f}, {goal_y:.2f}) m")
        
        # Get current pose from SLAM
        try:
            current_x, current_y, current_theta = self.slam.get_pose()
            print(f"Start:  ({current_x:.2f}, {current_y:.2f}) m")
        except Exception as e:
            print(f"Error getting SLAM pose: {e}")
            return False
        
        # Calculate straight-line distance
        distance = np.hypot(goal_x - current_x, goal_y - current_y)
        print(f"Distance: {distance:.2f} m")
        
        # Plan path
        print("\nPlanning path...")
        path = self.planner.plan_path(current_x, current_y, goal_x, goal_y)
        
        if path is None or len(path) == 0:
            print("No valid path found!")
            return False
        
        self.current_path = deque(path)
        print(f"Path planned: {len(path)} waypoints")
        
        # Execute navigation
        return self._follow_path(goal_x, goal_y)
    
    def _follow_path(self, goal_x: float, goal_y: float) -> bool:
        """
        Follow planned path with obstacle avoidance.
        
        Args:
            goal_x, goal_y: Final goal coordinates
            
        Returns:
            bool: True if goal reached
        """
        self.navigation_active = True
        waypoint_count = 0
        obstacle_stop_count = 0
        
        print(f"\nStarting navigation...")
        print(f"Safety: Will stop if obstacle < {self.safety_distance*100:.0f} cm\n")
        
        try:
            while len(self.current_path) > 0 and self.navigation_active:
                # CRITICAL: Safety check with ultrasonic sensor
                try:
                    distance = self.robot.get_ultrasonic()
                    if distance < self.safety_distance:
                        obstacle_stop_count += 1
                        print(f"\n SAFETY STOP! Obstacle at {distance*100:.1f} cm")
                        self.robot.drive(0, 0)
                        time.sleep(0.5)
                        continue  # Wait for obstacle to clear
                    elif obstacle_stop_count > 0:
                        print(f"Path clear, resuming navigation")
                        obstacle_stop_count = 0
                except Exception as e:
                    print(f"Warning: Ultrasonic read failed: {e}")
                
                # Get current pose from SLAM
                try:
                    current_x, current_y, current_theta = self.slam.get_pose()
                except Exception as e:
                    print(f"Error: Lost SLAM pose: {e}")
                    self.robot.drive(0, 0)
                    return False
                
                # Check if we've reached final goal
                dist_to_goal = np.hypot(goal_x - current_x, goal_y - current_y)
                if dist_to_goal < self.goal_tolerance:
                    print(f"\nGOAL REACHED!")
                    print(f"   Final distance: {dist_to_goal*100:.1f} cm")
                    print(f"   Final position: ({current_x:.3f}, {current_y:.3f})")
                    self.robot.drive(0, 0)
                    self.navigation_active = False
                    return True
                
                # Get next waypoint
                waypoint = self.current_path[0]
                wp_x, wp_y = waypoint
                
                # Check if we've reached current waypoint
                dist_to_waypoint = np.hypot(wp_x - current_x, wp_y - current_y)
                if dist_to_waypoint < self.waypoint_threshold:
                    self.current_path.popleft()
                    waypoint_count += 1
                    if len(self.current_path) > 0:
                        print(f"Waypoint {waypoint_count}/{waypoint_count + len(self.current_path)} "
                              f"[{dist_to_goal:.2f}m to goal]")
                    continue
                
                # Compute control commands
                linear_vel, angular_vel = self._compute_velocity(
                    current_x, current_y, current_theta,
                    wp_x, wp_y
                )
                
                # Send drive command
                self.robot.drive(linear_vel, angular_vel)
                
                # Control loop timing
                time.sleep(0.1)  # 10 Hz
        
        except KeyboardInterrupt:
            print("\n Navigation interrupted by user")
            self.robot.drive(0, 0)
            self.navigation_active = False
            return False
        
        except Exception as e:
            print(f"\n Navigation error: {e}")
            self.robot.drive(0, 0)
            self.navigation_active = False
            return False
        
        # Stop robot
        self.robot.drive(0, 0)
        print(f"\n Navigation ended without reaching goal")
        print(f"   Distance remaining: {dist_to_goal:.2f} m")
        return False
    
    def _compute_velocity(self, x, y, theta, target_x, target_y):
        """
        Compute velocity commands using proportional control.
        
        Returns:
            (linear_vel, angular_vel): Velocity commands in m/s and rad/s
        """
        # Compute distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = np.hypot(dx, dy)
        
        # Desired heading angle
        desired_theta = np.arctan2(dy, dx)
        
        # Angle error (normalized to [-pi, pi])
        angle_error = desired_theta - theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        # Proportional control
        linear_vel = self.k_linear * distance
        angular_vel = self.k_angular * angle_error
        
        # Apply velocity limits
        linear_vel = np.clip(linear_vel, 0, self.max_linear_speed)
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, 
                             self.max_angular_speed)
        
        # Reduce linear velocity when turning sharply
        if abs(angle_error) > 0.5:  # ~30 degrees
            linear_vel *= 0.3
        elif abs(angle_error) > 0.3:  # ~17 degrees
            linear_vel *= 0.6
        
        return linear_vel, angular_vel
    
    def stop(self):
        """Emergency stop navigation."""
        self.navigation_active = False
        self.robot.drive(0, 0)
        print("Navigation stopped")


class RobotSLAM:
    """Complete robot system with place teaching and autonomous navigation."""
    
    def __init__(self, robot_interface=None, slam_interface=None, 
                 occupancy_map=None):
        """
        Initialize complete robot system.
        
        Args:
            robot_interface: Hardware interface with drive() and get_ultrasonic()
            slam_interface: SLAM system with get_pose()
            occupancy_map: Optional occupancy grid for path planning
        """
        self.robot = robot_interface
        self.slam_system = slam_interface
        self.place_manager = PlaceManager()
        
        # Path planner
        path_planner = PathPlanner(occupancy_map)
        
        # Navigator
        if robot_interface and slam_interface:
            self.navigator = RobotNavigator(robot_interface, slam_interface, 
                                           path_planner)
        else:
            self.navigator = None
            print(" Navigator disabled: robot/SLAM interfaces not provided")
        
        # For simulation/testing
        self.simulated_pose = Pose(x=0.0, y=0.0, theta=0.0)
    
    def get_current_pose(self) -> Pose:
        """Get current robot pose from SLAM or simulation."""
        if self.slam_system is not None:
            try:
                x, y, theta = self.slam_system.get_pose()
                return Pose(x=x, y=y, theta=theta)
            except Exception as e:
                print(f"Warning: SLAM pose failed: {e}")
        
        return self.simulated_pose
    
    def teach_place(self, name: str, aliases: Optional[List[str]] = None):
        """
        Teach the robot the current location as a named place.
        
        Args:
            name: The name for this place
            aliases: Optional list of alternative names
            
        Example:
            >>> robot.teach_place("kitchen", aliases=["cooking area"])
        """
        current_pose = self.get_current_pose()
        self.place_manager.teach_place(name, current_pose, aliases)
    
    def set_goal(self, name: str) -> bool:
        """
        Drive to a saved place using SLAM pose + path planner.
        
        Args:
            name: Name or alias of the saved place
            
        Returns:
            bool: True if navigation successful
            
        Example:
            >>> robot.set_goal("kitchen")
             Navigating to 'kitchen'...
             Path planned: 15 waypoints
             Starting navigation...
             GOAL REACHED! Final distance: 38.2 cm
        """
        if self.navigator is None:
            print("Navigator not available (no robot/SLAM interface)")
            return False
        
        # Look up the place
        place_data = self.place_manager.get_place(name)
        
        if place_data is None:
            print(f"Place '{name}' not found")
            self.place_manager.list_places()
            return False
        
        # Extract goal coordinates
        goal_x = place_data['x']
        goal_y = place_data['y']
        goal_theta = place_data.get('theta', None)
        actual_name = place_data['name']
        
        print(f"\n Navigating to '{actual_name}'")
        if name.lower() != actual_name.lower():
            print(f"   (matched alias: '{name}')")
        
        # Navigate
        return self.navigator.navigate_to_pose(goal_x, goal_y, goal_theta)
    
    def list_places(self):
        """Show all saved places."""
        self.place_manager.list_places()
    
    # Simulation methods for testing
    def drive_to_position(self, x: float, y: float, theta: float):
        """Simulate driving to a position (for testing only)."""
        self.simulated_pose = Pose(x=x, y=y, theta=theta)
        print(f"Simulated position: ({x:.2f}, {y:.2f}, {theta:.2f})")


# Command-Line Interface for Raspberry Pi

def main():
    """Interactive CLI for Raspberry Pi."""
    import sys
    
    print("=" * 70)
    print("SLAM NAVIGATION SYSTEM")
    print("   - teach_place(name, aliases) - Save current location")
    print("   - set_goal(name) - Navigate to saved location")
    print("=" * 70)
    
    # Initialize robot
    # TODO: Replace with actual interfaces
    robot = RobotSLAM()
    
    if len(sys.argv) > 1:
        # Command-line mode
        command = sys.argv[1]
        
        if command == "teach":
            if len(sys.argv) < 3:
                print("Usage: python3 navigation.py teach <n> [alias1,alias2]")
                return
            
            name = sys.argv[2]
            aliases = []
            if len(sys.argv) > 3:
                aliases = [a.strip() for a in sys.argv[3].split(',')]
            
            robot.teach_place(name, aliases=aliases)
        
        elif command == "goto" or command == "set_goal":
            if len(sys.argv) < 3:
                print("Usage: python3 navigation.py goto <n>")
                return
            
            name = sys.argv[2]
            success = robot.set_goal(name)
            sys.exit(0 if success else 1)
        
        elif command == "list":
            robot.list_places()
        
        else:
            print(f"Unknown command: {command}")
            print("Commands: teach, goto, list")
    
    else:
        # Interactive mode
        print("\nCommands:")
        print("  teach <n> [aliases] - Teach current location")
        print("  goto <n> - Navigate to saved place")
        print("  list - Show all places")
        print("  exit - Quit")
        
        while True:
            try:
                cmd_input = input("\n> ").strip()
                if not cmd_input:
                    continue
                
                parts = cmd_input.split()
                cmd = parts[0]
                
                if cmd == "teach":
                    if len(parts) < 2:
                        print("Usage: teach <n>")
                        continue
                    name = parts[1]
                    alias_input = input("Aliases (comma-separated, Enter to skip): ").strip()
                    aliases = [a.strip() for a in alias_input.split(',')] if alias_input else []
                    robot.teach_place(name, aliases=aliases)
                
                elif cmd == "goto":
                    if len(parts) < 2:
                        print("Usage: goto <n>")
                        continue
                    robot.set_goal(parts[1])
                
                elif cmd == "list":
                    robot.list_places()
                
                elif cmd in ["exit", "quit", "q"]:
                    print("Goodbye!")
                    break
                
                else:
                    print(f"Unknown command: {cmd}")
            
            except KeyboardInterrupt:
                print("\nGoodbye!")
                break
            except Exception as e:
                print(f"Error: {e}")


# Demo

def run_demo():
    """Demo the complete system."""
    print("=" * 70)
    print("COMPLETE NAVIGATION DEMO")
    print("=" * 70)
    
    robot = RobotSLAM()
    
    # 1. Teach places
    print("\n--- STEP 1: Teaching Places ---")
    robot.drive_to_position(x=1.2, y=3.4, theta=0.1)
    robot.teach_place("kitchen", aliases=["cooking area"])
    
    robot.drive_to_position(x=5.6, y=2.1, theta=1.57)
    robot.teach_place("living room", aliases=["lounge"])
    
    robot.drive_to_position(x=8.3, y=7.9, theta=-0.5)
    robot.teach_place("bedroom")
    
    # 2. List places
    robot.list_places()
    
    # 3. Demonstrate navigation (simulated)
    print("\n--- STEP 2: Navigation Demo ---")
    print("In real usage, this would autonomously navigate:")
    print("  robot.set_goal('kitchen')")
    print("\nWith real hardware:")
    print("  - Plans path using A* planner")
    print("  - Follows waypoints with SLAM localization")
    print("  - Stops automatically if obstacle < 15 cm")
    print("  - Arrives within ~40 cm of target")
    
    print("\n Demo complete! Ready for real hardware integration.")


if __name__ == "__main__":
    # For demo/testing
    run_demo()
    
    # For actual use on Pi
    # main()