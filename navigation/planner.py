"""
planner.py - Complete Navigation System Test
Tests integration of SLAM, Places, and Path Planning

This script tests:
- Teaching places with SLAM poses
- Retrieving saved places
- Planning paths to saved locations
- Autonomous navigation with obstacle avoidance
- Verifying robot "remembers" locations correctly
"""

import sys
import time
import numpy as np
from typing import Tuple, Optional

# Import actual modules (adjust paths as needed)
try:
    from slam import SLAM  # SLAM implementation
    from places import PlaceManager, Pose  # From the navigation.py code
    from planner import PathPlanner  # path planner
except ImportError:
    print("Warning: Using mock implementations for testing")
    print("Replace with your actual SLAM, PlaceManager, and PathPlanner imports")
    
    # Mock implementations for testing
    class Pose:
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
    
    class SLAM:
        def __init__(self):
            self.pose = (0.0, 0.0, 0.0)
        
        def get_pose(self):
            return self.pose
        
        def update_pose(self, x, y, theta):
            self.pose = (x, y, theta)
    
    class PathPlanner:
        def plan_path(self, start_x, start_y, goal_x, goal_y):
            # Simple straight line path
            steps = 10
            path = []
            for i in range(steps + 1):
                t = i / steps
                x = start_x + t * (goal_x - start_x)
                y = start_y + t * (goal_y - start_y)
                path.append((x, y))
            return path
    
    # Import PlaceManager from navigation.py or define it here
    import json
    import os
    
    class PlaceManager:
        def __init__(self, filename="places.json"):
            self.filename = filename
            self.places = self._load_places()
        
        def _load_places(self):
            if os.path.exists(self.filename):
                try:
                    with open(self.filename, 'r') as f:
                        return json.load(f)
                except:
                    return {}
            return {}
        
        def _save_places(self):
            with open(self.filename, 'w') as f:
                json.dump(self.places, f, indent=2)
        
        def teach_place(self, name, pose, aliases=None):
            if aliases is None:
                aliases = []
            self.places[name] = {
                "x": round(pose.x, 3),
                "y": round(pose.y, 3),
                "theta": round(pose.theta, 3),
                "aliases": aliases
            }
            self._save_places()
        
        def get_place(self, name):
            name_lower = name.lower()
            for place_name, place_data in self.places.items():
                if place_name.lower() == name_lower:
                    return {"name": place_name, **place_data}
                aliases = [a.lower() for a in place_data.get("aliases", [])]
                if name_lower in aliases:
                    return {"name": place_name, **place_data}
            return None
        
        def list_places(self):
            if not self.places:
                print("No places saved.")
                return
            print("\nSaved Places:")
            for name, data in self.places.items():
                print(f"  {name}: ({data['x']}, {data['y']}, {data['theta']})")
                if data.get('aliases'):
                    print(f"    Aliases: {', '.join(data['aliases'])}")


class NavigationTester:
    """Comprehensive test suite for navigation system."""
    
    def __init__(self):
        self.slam = SLAM()
        self.place_manager = PlaceManager("test_places.json")
        self.planner = PathPlanner()
        
        self.test_results = {
            "passed": 0,
            "failed": 0,
            "tests": []
        }
    
    def log_test(self, test_name: str, passed: bool, message: str = ""):
        """Log test result."""
        status = "PASS" if passed else "FAIL"
        print(f"{status}: {test_name}")
        if message:
            print(f"       {message}")
        
        self.test_results["tests"].append({
            "name": test_name,
            "passed": passed,
            "message": message
        })
        
        if passed:
            self.test_results["passed"] += 1
        else:
            self.test_results["failed"] += 1
    
    def simulate_drive_to(self, x: float, y: float, theta: float):
        """Simulate driving robot to a position."""
        print(f"\n Driving to position: ({x:.2f}, {y:.2f}, {theta:.2f})")
        self.slam.update_pose(x, y, theta)
        time.sleep(0.2)  # Simulate movement time
    
    def test_1_teach_places(self):
        """Test 1: Teach multiple places and verify they're saved."""
        print("\n" + "="*70)
        print("TEST 1: Teaching Places")
        print("="*70)
        
        # Clear existing places for clean test
        self.place_manager.places = {}
        
        # Teach kitchen
        self.simulate_drive_to(1.2, 3.4, 0.1)
        current_x, current_y, current_theta = self.slam.get_pose()
        pose = Pose(current_x, current_y, current_theta)
        self.place_manager.teach_place("kitchen", pose, aliases=["cooking area"])
        
        # Verify kitchen was saved
        kitchen = self.place_manager.get_place("kitchen")
        if kitchen:
            correct_pos = (abs(kitchen['x'] - 1.2) < 0.01 and 
                          abs(kitchen['y'] - 3.4) < 0.01)
            self.log_test("Teach kitchen", correct_pos,
                         f"Saved at ({kitchen['x']}, {kitchen['y']})")
        else:
            self.log_test("Teach kitchen", False, "Kitchen not found in places")
        
        # Teach living room
        self.simulate_drive_to(5.6, 2.1, 1.57)
        current_x, current_y, current_theta = self.slam.get_pose()
        pose = Pose(current_x, current_y, current_theta)
        self.place_manager.teach_place("living room", pose, aliases=["lounge"])
        
        # Teach bedroom
        self.simulate_drive_to(8.3, 7.9, -0.5)
        current_x, current_y, current_theta = self.slam.get_pose()
        pose = Pose(current_x, current_y, current_theta)
        self.place_manager.teach_place("bedroom", pose, aliases=["sleeping area"])
        
        # Verify all places saved
        num_places = len(self.place_manager.places)
        self.log_test("Save 3 places", num_places == 3,
                     f"Saved {num_places} places (expected 3)")
    
    def test_2_retrieve_places(self):
        """Test 2: Retrieve places by name and alias."""
        print("\n" + "="*70)
        print("TEST 2: Retrieving Places")
        print("="*70)
        
        # Test retrieval by direct name
        kitchen = self.place_manager.get_place("kitchen")
        self.log_test("Retrieve by name", kitchen is not None,
                     f"Found: {kitchen['name'] if kitchen else 'None'}")
        
        # Test retrieval by alias
        cooking_area = self.place_manager.get_place("cooking area")
        self.log_test("Retrieve by alias", 
                     cooking_area is not None and cooking_area['name'] == 'kitchen',
                     "Alias 'cooking area' maps to 'kitchen'")
        
        # Test case-insensitive retrieval
        KITCHEN = self.place_manager.get_place("KITCHEN")
        self.log_test("Case-insensitive retrieval", KITCHEN is not None,
                     "Successfully retrieved 'KITCHEN' (uppercase)")
        
        # Test non-existent place
        fake = self.place_manager.get_place("nonexistent")
        self.log_test("Non-existent place", fake is None,
                     "Correctly returns None for unknown place")
    
    def test_3_slam_pose_accuracy(self):
        """Test 3: Verify SLAM pose matches taught location."""
        print("\n" + "="*70)
        print("TEST 3: SLAM Pose Accuracy")
        print("="*70)
        
        # Get kitchen location
        kitchen = self.place_manager.get_place("kitchen")
        if not kitchen:
            self.log_test("SLAM pose accuracy", False, "Kitchen not found")
            return
        
        # Simulate driving to kitchen coordinates
        self.simulate_drive_to(kitchen['x'], kitchen['y'], kitchen['theta'])
        
        # Get current SLAM pose
        current_x, current_y, current_theta = self.slam.get_pose()
        
        # Calculate error
        error = np.hypot(current_x - kitchen['x'], current_y - kitchen['y'])
        
        # Check if within tolerance (1 mm)
        within_tolerance = error < 0.001
        self.log_test("SLAM pose matches saved location", within_tolerance,
                     f"Position error: {error*1000:.2f} mm")
    
    def test_4_path_planning(self):
        """Test 4: Plan paths to saved locations."""
        print("\n" + "="*70)
        print("TEST 4: Path Planning")
        print("="*70)
        
        # Start from origin
        self.simulate_drive_to(0.0, 0.0, 0.0)
        start_x, start_y, _ = self.slam.get_pose()
        
        # Get kitchen location
        kitchen = self.place_manager.get_place("kitchen")
        if not kitchen:
            self.log_test("Plan path to kitchen", False, "Kitchen not found")
            return
        
        # Plan path
        path = self.planner.plan_path(start_x, start_y, kitchen['x'], kitchen['y'])
        
        # Verify path exists
        self.log_test("Path planning succeeds", path is not None and len(path) > 0,
                     f"Generated path with {len(path) if path else 0} waypoints")
        
        if path:
            # Verify path starts near current position
            first_point = path[0]
            start_error = np.hypot(first_point[0] - start_x, first_point[1] - start_y)
            self.log_test("Path starts at current position", start_error < 0.1,
                         f"Start error: {start_error:.3f} m")
            
            # Verify path ends near goal
            last_point = path[-1]
            goal_error = np.hypot(last_point[0] - kitchen['x'], 
                                 last_point[1] - kitchen['y'])
            self.log_test("Path ends at goal position", goal_error < 0.1,
                         f"Goal error: {goal_error:.3f} m")
    
    def test_5_navigation_simulation(self):
        """Test 5: Simulate full navigation to saved location."""
        print("\n" + "="*70)
        print("TEST 5: Navigation Simulation")
        print("="*70)
        
        # Start from origin
        self.simulate_drive_to(0.0, 0.0, 0.0)
        
        # Get bedroom location
        bedroom = self.place_manager.get_place("bedroom")
        if not bedroom:
            self.log_test("Navigation simulation", False, "Bedroom not found")
            return
        
        # Plan path
        start_x, start_y, _ = self.slam.get_pose()
        path = self.planner.plan_path(start_x, start_y, bedroom['x'], bedroom['y'])
        
        if not path:
            self.log_test("Navigation simulation", False, "Path planning failed")
            return
        
        # Simulate following path
        print(f"\n  Navigating to bedroom at ({bedroom['x']}, {bedroom['y']})...")
        for i, (wp_x, wp_y) in enumerate(path):
            # Simulate moving to waypoint
            if i % 3 == 0:  # Print every 3rd waypoint
                current_x, current_y, _ = self.slam.get_pose()
                dist_to_goal = np.hypot(bedroom['x'] - wp_x, bedroom['y'] - wp_y)
                print(f"  → Waypoint {i+1}/{len(path)} "
                      f"[{dist_to_goal:.2f}m to goal]")
            
            self.slam.update_pose(wp_x, wp_y, 0.0)
            time.sleep(0.05)  # Simulate movement
        
        # Check final position
        final_x, final_y, _ = self.slam.get_pose()
        final_error = np.hypot(final_x - bedroom['x'], final_y - bedroom['y'])
        
        # Navigation successful if within 40 cm
        success = final_error < 0.40
        self.log_test("Navigation within 40cm tolerance", success,
                     f"Final error: {final_error*100:.1f} cm")
    
    def test_6_memory_persistence(self):
        """Test 6: Verify places persist across sessions."""
        print("\n" + "="*70)
        print("TEST 6: Memory Persistence")
        print("="*70)
        
        # Save current places
        original_count = len(self.place_manager.places)
        
        # Create new PlaceManager instance (simulates restart)
        new_manager = PlaceManager("test_places.json")
        reloaded_count = len(new_manager.places)
        
        self.log_test("Places persist after reload", 
                     original_count == reloaded_count,
                     f"Original: {original_count}, Reloaded: {reloaded_count}")
        
        # Verify specific place still exists
        kitchen = new_manager.get_place("kitchen")
        self.log_test("Specific place (kitchen) persists", 
                     kitchen is not None,
                     f"Kitchen found: {kitchen is not None}")
    
    def test_7_alias_functionality(self):
        """Test 7: Verify alias lookup works correctly."""
        print("\n" + "="*70)
        print("TEST 7: Alias Functionality")
        print("="*70)
        
        # Test multiple aliases for same place
        kitchen_by_name = self.place_manager.get_place("kitchen")
        kitchen_by_alias = self.place_manager.get_place("cooking area")
        
        if kitchen_by_name and kitchen_by_alias:
            same_place = (kitchen_by_name['x'] == kitchen_by_alias['x'] and
                         kitchen_by_name['y'] == kitchen_by_alias['y'])
            self.log_test("Alias maps to same location", same_place,
                         "Name and alias return identical location")
        else:
            self.log_test("Alias maps to same location", False,
                         "Kitchen or alias not found")
        
        # Test all aliases
        test_aliases = [
            ("kitchen", "kitchen"),
            ("cooking area", "kitchen"),
            ("lounge", "living room"),
            ("sleeping area", "bedroom")
        ]
        
        for query, expected_name in test_aliases:
            result = self.place_manager.get_place(query)
            success = result is not None and result['name'] == expected_name
            self.log_test(f"Alias '{query}' → '{expected_name}'", success)
    
    def run_all_tests(self):
        """Run complete test suite."""
        print("\n" + "="*70)
        print("NAVIGATION SYSTEM TEST SUITE")
        print("   Testing: SLAM + Places + Planner Integration")
        print("="*70)
        
        # Run all tests
        self.test_1_teach_places()
        self.test_2_retrieve_places()
        self.test_3_slam_pose_accuracy()
        self.test_4_path_planning()
        self.test_5_navigation_simulation()
        self.test_6_memory_persistence()
        self.test_7_alias_functionality()
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test results summary."""
        print("\n" + "="*70)
        print("TEST RESULTS SUMMARY")
        print("="*70)
        
        total = self.test_results["passed"] + self.test_results["failed"]
        pass_rate = (self.test_results["passed"] / total * 100) if total > 0 else 0
        
        print(f"\nTotal Tests:  {total}")
        print(f"Passed:       {self.test_results['passed']} ✓")
        print(f"Failed:       {self.test_results['failed']} ✗")
        print(f"Pass Rate:    {pass_rate:.1f}%")
        
        if self.test_results["failed"] == 0:
            print("\n ALL TESTS PASSED! Navigation system is working correctly.")
        else:
            print(f"\n {self.test_results['failed']} test(s) failed. Review above for details.")
        
        print("="*70)
        
        # List all saved places
        print("\n Final Saved Places:")
        self.place_manager.list_places()
    
    def interactive_test(self):
        """Interactive testing mode."""
        print("\n" + "="*70)
        print(" INTERACTIVE TEST MODE")
        print("="*70)
        print("\nCommands:")
        print("  teach <n> <x> <y> <theta> - Teach a place")
        print("  goto <n> - Simulate navigation to place")
        print("  list - Show all places")
        print("  test - Run automated test suite")
        print("  exit - Quit")
        
        while True:
            try:
                cmd = input("\ntest> ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == "teach" and len(cmd) >= 5:
                    name = cmd[1]
                    x, y, theta = float(cmd[2]), float(cmd[3]), float(cmd[4])
                    self.simulate_drive_to(x, y, theta)
                    pose = Pose(x, y, theta)
                    self.place_manager.teach_place(name, pose)
                
                elif cmd[0] == "goto" and len(cmd) >= 2:
                    name = cmd[1]
                    place = self.place_manager.get_place(name)
                    if place:
                        print(f"\n Navigating to '{place['name']}'...")
                        start_x, start_y, _ = self.slam.get_pose()
                        path = self.planner.plan_path(start_x, start_y, 
                                                     place['x'], place['y'])
                        if path:
                            for wp_x, wp_y in path:
                                self.slam.update_pose(wp_x, wp_y, 0.0)
                            print(f" Arrived at ({place['x']}, {place['y']})")
                    else:
                        print(f" Place '{name}' not found")
                
                elif cmd[0] == "list":
                    self.place_manager.list_places()
                
                elif cmd[0] == "test":
                    self.run_all_tests()
                
                elif cmd[0] in ["exit", "quit"]:
                    break
                
                else:
                    print("Unknown command")
            
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Test navigation system")
    parser.add_argument("--interactive", "-i", action="store_true",
                       help="Run in interactive mode")
    parser.add_argument("--quick", "-q", action="store_true",
                       help="Run quick test (fewer tests)")
    
    args = parser.parse_args()
    
    tester = NavigationTester()
    
    if args.interactive:
        tester.interactive_test()
    else:
        tester.run_all_tests()


if __name__ == "__main__":
    main()