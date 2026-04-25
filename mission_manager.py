"""
Swayam Mission Manager
Handles loading, parsing, and execution of multi-drone mission files.
"""

import json
import time

class MissionManager:
    def __init__(self, bridge):
        self.bridge = bridge
        self.current_mission = []
        self.mission_index = 0
        self.is_running = False

    def load_mission(self, file_path):
        """Loads a mission from a JSON file."""
        try:
            with open(file_path, 'r') as f:
                self.current_mission = json.load(f)
            print(f"[MISSION] Loaded {len(self.current_mission)} waypoints.")
            return True
        except Exception as e:
            print(f"[ERR] Failed to load mission: {e}")
            return False

    def start_mission(self):
        """Begins execution of the loaded mission."""
        if not self.current_mission:
            print("[MISSION] No mission loaded.")
            return
        
        self.is_running = True
        self.mission_index = 0
        print("[MISSION] Starting execution...")

    def update(self):
        """Should be called in a loop to process waypoints."""
        if not self.is_running or self.mission_index >= len(self.current_mission):
            self.is_running = False
            return False

        wp = self.current_mission[self.mission_index]
        print(f"[MISSION] Heading to WP {self.mission_index}: {wp}")
        
        # Send to bridge
        self.bridge.send_global_position_int(wp['lat'], wp['lon'], wp['alt'], wp['alt'])
        
        # In a real system, we'd check distance to WP before incrementing
        # Here we simulate with a sleep/wait logic
        self.mission_index += 1
        return True

if __name__ == "__main__":
    # Example mission structure
    example_mission = [
        {"lat": 12.9716, "lon": 77.5946, "alt": 10},
        {"lat": 12.9720, "lon": 77.5950, "alt": 15},
        {"lat": 12.9710, "lon": 77.5960, "alt": 10}
    ]
    with open("example_mission.json", "w") as f:
        json.dump(example_mission, f)
