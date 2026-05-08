# the actual wire talking to the cube orange
# if the serial port disconnects mid-flight, rip

import time
from pymavlink import mavutil
from src.swayam.hardware.hw import get_connection_string

class MavlinkBridge:

    def __init__(self, connection=None):
        if connection is None:
            connection = get_connection_string()
        print(f'[BRIDGE] Connecting to {connection}...')
        self.master = mavutil.mavlink_connection(connection)
        self.master.wait_heartbeat()
        print(f'[BRIDGE] Connected to System {self.master.target_system}')

    def arm_disarm(self, arm=True):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if arm else 0, 0, 0, 0, 0, 0, 0)
        print(f"[BRIDGE] {('Armed' if arm else 'Disarmed')}")

    def set_mode(self, mode):
        if mode not in self.master.mode_mapping():
            print(f'[BRIDGE] Mode {mode} not found.')
            return
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        print(f'[BRIDGE] Mode set to {mode}')

    def send_global_position_int(self, lat, lon, alt, relative_alt):
        # hardware hack: Pi4 UART buffer overflows if we blast it too fast
        # without this the cube orange just drops packets silently
        time.sleep(0.02)
        self.master.mav.set_position_target_global_int_send(0, self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 4088, int(lat * 10000000.0), int(lon * 10000000.0), alt, 0, 0, 0, 0, 0, 0, 0, 0)
if __name__ == '__main__':
    try:
        bridge = MavlinkBridge('udp:127.0.0.1:14550')
        bridge.arm_disarm(True)
        time.sleep(1)
        bridge.set_mode('GUIDED')
    except Exception as e:
        print(f'[ERR] Bridge test failed: {e}')
