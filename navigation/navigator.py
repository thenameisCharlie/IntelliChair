from navigation.places import PlaceManager
from navigation.planner import SimplePlanner

class Navigator:
    """
    Thin wrapper exposing:
      - go_to(name): navigate to a saved waypoint
      - emergency_stop(): immediate halt via planner abort flag + motor stop
      - current_room(): simple tracker for status replies
    """
    def __init__(self):
        self.pm = PlaceManager()
        self.planner = SimplePlanner()
        self._current_room = "living room"

    def go_to(self, name: str):
        place = self.pm.get_place(name)
        if place is None:
            raise ValueError(f"Unknown place '{name}'. Known: {self.pm.list_places()}")
        self._current_room = name.lower()
        # Clear any previous abort before starting a new goal
        try:
            self.planner.abort.clear()
        except Exception:
            pass
        self.planner.set_goal(name)

    def emergency_stop(self):
        # Signal planner to abort and stop motors immediately
        try:
            self.planner.abort.set()
        except Exception:
            pass
        try:
            self.planner.motors.stop()
        except Exception:
            pass

    def current_room(self) -> str:
        return self._current_room
