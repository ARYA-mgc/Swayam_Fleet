"""
Swayam Autonomous Swarm Logic
Handles decision making based on swarm telemetry and commands.
"""

import math

class AutonomousSwarmLogic:
    def __init__(self, my_id):
        self.my_id = my_id
        self.swarm_state = {} # ID -> last status

    def update_swarm_state(self, drone_id, data):
        """Updates internal state with data from another drone."""
        if drone_id != self.my_id:
            self.swarm_state[drone_id] = data

    def calculate_follow_offset(self, leader_id, offset_n=2.0, offset_e=2.0):
        """Calculates a target position to follow a leader with an offset."""
        if leader_id not in self.swarm_state:
            return None
        
        leader_pos = self.swarm_state[leader_id]
        target_n = leader_pos.get("lat") + (offset_n / 111111.0) # Very rough deg conversion
        target_e = leader_pos.get("lon") + (offset_e / (111111.0 * math.cos(math.radians(leader_pos.get("lat")))))
        
        return (target_n, target_e, leader_pos.get("alt"))

    def check_collision_risk(self, my_pos, safety_radius=5.0):
        """Checks if any other drone is within the safety radius."""
        my_lat, my_lon = my_pos
        for drone_id, data in self.swarm_state.items():
            dist = self._haversine(my_lat, my_lon, data['lat'], data['lon'])
            if dist < safety_radius:
                return drone_id # Collision risk with this drone
        return None

    def _haversine(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points in meters."""
        R = 6371000 # Radius of Earth
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
