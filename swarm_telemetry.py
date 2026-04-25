"""
Swayam Swarm Telemetry Aggregator
Collects and broadcasts telemetry across the swarm network.
"""

import time
import json
import socket

class SwarmTelemetry:
    def __init__(self, drone_id, broadcast_port=5555):
        self.drone_id = drone_id
        self.port = broadcast_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def broadcast_status(self, lat, lon, alt, battery, status):
        """Broadcasts current drone status to the swarm."""
        data = {
            "id": self.drone_id,
            "ts": time.time(),
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "bat": battery,
            "stat": status
        }
        msg = json.dumps(data).encode('utf-8')
        self.sock.sendto(msg, ('<broadcast>', self.port))
        print(f"[TELEMETRY] Broadcasted {self.drone_id} status.")

    def listen_swarm(self):
        """Listens for other drones in the swarm."""
        self.sock.bind(('', self.port))
        print(f"[TELEMETRY] Listening on port {self.port}...")
        while True:
            data, addr = self.sock.recvfrom(1024)
            remote_data = json.loads(data.decode('utf-8'))
            print(f"[SWARM INFO] Received from {remote_data['id']}: {remote_data['stat']}")

if __name__ == "__main__":
    tel = SwarmTelemetry("ALPHA")
    tel.broadcast_status(12.9716, 77.5946, 10.0, 95, "ACTIVE")
