"""
Swayam Swarm GCS Relay
======================
Aggregates MAVLink traffic from multiple drones into a single GCS connection.
This allows Mission Planner to see the entire swarm through one UDP port.
"""

import threading
import time
import logging
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("swayam.relay")

class SwarmGCSRelay:
    def __init__(self, gcs_url="udpout:127.0.0.1:14550"):
        self.gcs_url = gcs_url
        self.drones = {} # map sysid -> connection_string
        self.drone_conns = {} # map sysid -> mavutil connection
        self.gcs_conn = None
        self._running = False
        self._threads = []

    def add_drone(self, sysid, connection_string):
        self.drones[sysid] = connection_string
        logger.info(f"Relay: Registered Drone {sysid} at {connection_string}")

    def start(self):
        self._running = True
        logger.info(f"Relay: Starting GCS bridge at {self.gcs_url}")
        
        # Connect to GCS
        self.gcs_conn = mavutil.mavlink_connection(self.gcs_url, source_system=255)

        # Start GCS -> Drones listener
        t_gcs = threading.Thread(target=self._gcs_listener, daemon=True)
        t_gcs.start()
        self._threads.append(t_gcs)

        # Start Drone -> GCS listeners
        for sysid, conn_str in self.drones.items():
            t_drone = threading.Thread(target=self._drone_listener, args=(sysid, conn_str), daemon=True)
            t_drone.start()
            self._threads.append(t_drone)

    def _drone_listener(self, sysid, conn_str):
        logger.info(f"Relay: Listening to Drone {sysid}...")
        conn = mavutil.mavlink_connection(conn_str)
        self.drone_conns[sysid] = conn
        
        while self._running:
            try:
                msg = conn.recv_match(blocking=True, timeout=0.1)
                if msg:
                    # Forward all traffic from drone to GCS
                    self.gcs_conn.mav.send(msg)
            except Exception as e:
                logger.error(f"Relay: Drone {sysid} error: {e}")
                time.sleep(1)

    def _gcs_listener(self):
        while self._running:
            try:
                msg = self.gcs_conn.recv_match(blocking=True, timeout=0.1)
                if msg:
                    target_sys = getattr(msg, 'target_system', 0)
                    if target_sys == 0:
                        # Broadcast to all drones
                        for conn in self.drone_conns.values():
                            conn.mav.send(msg)
                    elif target_sys in self.drone_conns:
                        # Targeted message
                        self.drone_conns[target_sys].mav.send(msg)
            except Exception as e:
                logger.error(f"Relay: GCS error: {e}")
                time.sleep(1)

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1)
        if self.gcs_conn:
            self.gcs_conn.close()
        for conn in self.drone_conns.values():
            conn.close()

if __name__ == "__main__":
    # Example usage
    relay = SwarmGCSRelay("udpout:127.0.0.1:14550")
    relay.add_drone(1, "udpin:127.0.0.1:14551")
    relay.add_drone(2, "udpin:127.0.0.1:14552")
    relay.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        relay.stop()
