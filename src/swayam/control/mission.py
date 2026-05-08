# reads json files so the drones know where to go
# mostly written at 3am, parse errors mean your json is trash not my code

import json
import time

class MissionManager:

    def __init__(self, bridge):
        self.bridge = bridge
        self.current_mission = []
        self.mission_index = 0
        self.is_running = False

    def load_mission(self, file_path):
        try:
            with open(file_path, 'r') as f:
                self.current_mission = json.load(f)
            print(f'[MISSION] Loaded {len(self.current_mission)} waypoints.')
            return True
        except Exception as e:
            print(f'[ERR] Failed to load mission: {e}')
            print('you probably forgot a comma in the json again')
            return False

    def start_mission(self):
        if not self.current_mission:
            print('[MISSION] No mission loaded.')
            return
        self.is_running = True
        self.mission_index = 0
        print('[MISSION] Starting execution...')

    def update(self):
        if not self.is_running or self.mission_index >= len(self.current_mission):
            self.is_running = False
            return False
        wp = self.current_mission[self.mission_index]
        print(f'[MISSION] Heading to WP {self.mission_index}: {wp}')
        self.bridge.send_global_position_int(wp['lat'], wp['lon'], wp['alt'], wp['alt'])
        self.mission_index += 1
        return True
if __name__ == '__main__':
    example_mission = [{'lat': 12.9716, 'lon': 77.5946, 'alt': 10}, {'lat': 12.972, 'lon': 77.595, 'alt': 15}, {'lat': 12.971, 'lon': 77.596, 'alt': 10}]
    with open('example_mission.json', 'w') as f:
        json.dump(example_mission, f)
