import json
import os
from typing import List, Dict, Optional
from dataclasses import dataclass, asdict

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
                    return json.load(f)
            except json.JSONDecodeError:
                print(f"Warning: Could not parse {self.filename}, starting fresh")
                return {}
        return {}
    
    def _save_places(self):
        """Save places to JSON file."""
        with open(self.filename, 'w') as f:
            json.dump(self.places, f, indent=2)
        print(f"Places saved to {self.filename}")
    
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
        
        # Store the place with its pose and aliases
        self.places[name] = {
            "x": round(pose.x, 3),
            "y": round(pose.y, 3),
            "theta": round(pose.theta, 3),
            "aliases": aliases
        }
        
        self._save_places()
        print(f"✓ Taught place '{name}' at position (x={pose.x:.2f}, y={pose.y:.2f}, θ={pose.theta:.2f})")
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
        # Check direct name match
        if name in self.places:
            return self.places[name]
        
        # Check aliases
        for place_name, place_data in self.places.items():
            if name in place_data.get("aliases", []):
                return place_data
        
        return None
    
    def list_places(self):
        """Print all saved places."""
        if not self.places:
            print("No places saved yet.")
            return
        
        print("\nSaved Places:")
        print("-" * 60)
        for name, data in self.places.items():
            aliases_str = f" (aliases: {', '.join(data['aliases'])})" if data.get('aliases') else ""
            print(f"{name}{aliases_str}")
            print(f"  Position: x={data['x']}, y={data['y']}, θ={data['theta']}")
        print("-" * 60)


class RobotSLAM:
    """Simulates a robot with SLAM capabilities."""
    
    def __init__(self):
        self.current_pose = Pose(x=0.0, y=0.0, theta=0.0)
        self.place_manager = PlaceManager()
    
    def get_current_pose(self) -> Pose:
        """Get the current robot pose from SLAM system."""
        # In a real implementation, this would interface with your SLAM system
        return self.current_pose
    
    def drive_to_position(self, x: float, y: float, theta: float):
        """
        Simulate driving the robot to a position.
        In real implementation, this would command the robot's navigation system.
        """
        self.current_pose = Pose(x=x, y=y, theta=theta)
        print(f"\n🤖 Robot moved to position: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
    
    def teach_place(self, name: str, aliases: Optional[List[str]] = None):
        """
        Teach the robot the current location as a named place.
        
        Args:
            name: The name for this place
            aliases: Optional list of alternative names
        """
        current_pose = self.get_current_pose()
        self.place_manager.teach_place(name, current_pose, aliases)


# Demo usage
if __name__ == "__main__":
    print("=" * 60)
    print("SLAM Place Teaching System Demo")
    print("=" * 60)
    
    # Create a robot instance
    robot = RobotSLAM()
    
    # Drive to kitchen and teach the location
    print("\n--- Teaching: Kitchen ---")
    robot.drive_to_position(x=1.2, y=3.4, theta=0.1)
    robot.teach_place("kitchen", aliases=["cooking area"])
    
    # Drive to living room and teach the location
    print("\n--- Teaching: Living Room ---")
    robot.drive_to_position(x=5.6, y=2.1, theta=1.57)
    robot.teach_place("living room", aliases=["lounge", "sitting area"])
    
    # Drive to bedroom and teach the location
    print("\n--- Teaching: Bedroom ---")
    robot.drive_to_position(x=8.3, y=7.9, theta=-0.5)
    robot.teach_place("bedroom", aliases=["sleeping area", "master bedroom"])
    
    # List all saved places
    robot.place_manager.list_places()
    
    # Demo retrieval
    print("\n--- Retrieving Places ---")
    kitchen = robot.place_manager.get_place("cooking area")
    if kitchen:
        print(f"Found 'cooking area': {kitchen}")
    
    print("\n✓ Demo complete! Check 'places.json' for saved data.")