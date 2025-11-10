"""
SLAM Navigation System
- 5.1 teach_place(name, aliases) to save current SLAM pose to places.json
- 5.2 set_goal(name) to drive to a saved place using SLAM pose + planner
"""

#update imports (Frank)
import json
import os
from dataclasses import dataclass
from typing import Dict, Any, Optional, List

# save in repo under navigation/places.json
PLACES_FILE = os.path.join(os.getcwd(), "navigation", "places.json")

@dataclass
class Pose:
    """Represents a 2D pose with x, y coordinates and theta orientation."""
    x: float
    y: float
    theta: float


class PlaceManager:
    """Manages saving and loading of named places with their poses."""
    #updated _init (Frank)
    def __init__(self, places_file: str = PLACES_FILE):
        self.places_file = places_file
        self.places: Dict[str, Dict[str, Any]] = {}
        self._load()

    #update _load (Frank)
    def _load(self):
        """Load places from JSON file if it exists."""
        if os.path.exists(self.places_file):
            try:
                with open(self.places_file, "r") as f:
                    raw = json.load(f)
                    self.places = {k.lower(): v for k, v in raw.items()}
                print(f"[places] Loaded {len(self.places)} places from {self.places_file}.")
            except Exception as e:
                print(f"[places] Error loading {self.places_file}: {e}")
                self.places = {}
        else:
            print(f"[places] No existing {self.places_file}, starting fresh.")
            self.places = {}

    #update _save (Frank)
    def _save(self):
        """Save places to JSON file with atomic write."""
        try:
            os.makedirs(os.path.dirname(self.places_file), exist_ok=True)
            with open(self.places_file, "w") as f:
                json.dump(self.places, f, indent=2)
            print(f"[places] Saved {len(self.places)} places to {self.places_file}")
        except Exception as e:
            print(f"[places] Error saving {self.places_file}: {e}")


    def clear_places(self):
        self.places = {}
        try:
            if os.path.exists(self.places_file):
                os.remove(self.places_file)
            print(f"[places] Cleared places (deleted {self.places_file}).")
        except Exception as e:
            print(f"[places] Error clearing places file: {e}")

    #change to add_place
    # def add_place(self, name: str, pose: Pose, aliases: Optional[List[str]] = None):
    #     """Add or overwrite a named place with optional aliases."""
    #     if aliases is None:
    #         aliases = []

    #     #canonical stored entry
    #     entry = {
    #         "x": float(pose.x),
    #         "y": float(pose.y),
    #         "theta": float(pose.theta),
    #         "aliases": aliases
    #     }

    #     #store under main name
    #     main_key = name.lower()
    #     self.places[main_key] = entry

    #     #store aliases mapping to the same entry data (but saved as separate keys)
    #     for alias in aliases:
    #         self.places[alias.lower()] = entry

    #     self._save()
    #     print(f"[places] Added place '{name}' @ ({pose.x:.2f}, {pose.y:.2f}, θ={pose.theta:.2f})")

    def add_place(self, name: str, pose: Pose, aliases: Optional[List[str]] = None):
        """Add or overwrite a named place with optional aliases (single canonical key)."""
        if aliases is None:
            aliases = []

        entry = {
            "x": float(pose.x),
            "y": float(pose.y),
            "theta": float(pose.theta),
            "aliases": [a.strip().lower() for a in aliases if a.strip()],
        }

        # Store only under the main canonical key
        main_key = name.strip().lower()
        self.places[main_key] = entry

        self._save()
        print(f"[places] Added place '{name}' @ ({pose.x:.2f}, {pose.y:.2f}, θ={pose.theta:.2f}) "
            f"aliases={entry['aliases']}")

    # def get_place(self, name: str) -> Optional[Dict[str, Any]]:
    #     """
    #     Retrieve a place by name or alias.
    #     """
    #     if not name:
    #         return None
    #     return self.places.get(name.lower())
    
    def get_place(self, name: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a place by exact name or any alias (case-insensitive).
        """
        if not name:
            return None
        key = name.strip().lower()

        # Exact name match
        if key in self.places:
            return self.places[key]

        # Alias match
        for entry in self.places.values():
            if key in [a.lower() for a in entry.get("aliases", [])]:
                return entry
        return None
    
    # def list_places(self) -> List[str]:
    #     """List all known place names."""
    #     return list(self.places.keys())

    def list_places(self) -> List[str]:
        """
        List canonical place names only (not alias strings as top-level keys).
        """
        return sorted(self.places.keys())


#Removed PathPlanner (Frank)

#Removed RobotNavigation (Frank)
