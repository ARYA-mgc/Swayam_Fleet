"""
Swayam MAVLink Bridge
Handles high-level communication between Raspberry Pi 4 and Pixhawk Cube Orange.
"""

import time
from pymavlink import mavutil
from pi_hardware_config import get_connection_string

class MavlinkBridge:
    def __init__(self, connection=None):
        if connection is None:
            connection = get_connection_string()
        
        print(f"[BRIDGE] Connecting to {connection}...")
        self.master = mavutil.mavlink_connection(connection)
        self.master.wait_heartbeat()
        print(f"[BRIDGE] Connected to System {self.master.target_system}")

    def arm_disarm(self, arm=True):
        """Arm or Disarm the drone."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if arm else 0, 0, 0, 0, 0, 0, 0
        )
        print(f"[BRIDGE] {'Armed' if arm else 'Disarmed'}")

    def set_mode(self, mode):
        """Set flight mode (e.g., GUIDED, LAND)."""
        if mode not in self.master.mode_mapping():
            print(f"[BRIDGE] Mode {mode} not found.")
            return
        
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"[BRIDGE] Mode set to {mode}")

    def send_global_position_int(self, lat, lon, alt, relative_alt):
        """Send target global position."""
        self.master.mav.set_position_target_global_int_send(
            0, # time_boot_ms
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000, # type_mask (only pos)
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, # velocity
            0, 0, 0, # acceleration
            0, 0 # yaw
        )

if __name__ == "__main__":
    # Test Bridge
    try:
        bridge = MavlinkBridge("udp:127.0.0.1:14550")
        bridge.arm_disarm(True)
        time.sleep(1)
        bridge.set_mode("GUIDED")
    except Exception as e:
        print(f"[ERR] Bridge test failed: {e}")
