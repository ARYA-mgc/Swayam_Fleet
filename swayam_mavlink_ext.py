"""
Swayam MAVLink Extension
Advanced MAVLink commands for swarm coordination and Cube Orange integration.
"""

from pymavlink import mavutil
import time

class SwayamMavlinkExt:
    def __init__(self, connection_string, baud=115200):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.master.wait_heartbeat()
        print(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")

    def set_message_interval(self, message_id, interval_us):
        """
        Set the interval at which a MAVLink message is sent.
        Useful for high-frequency IMU data for INS.
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0
        )

    def send_swarm_sync(self, drone_id, timestamp, status):
        """
        Sends a custom swarm status heartbeat (using STATUSTEXT as a placeholder or custom msg).
        """
        msg = f"SWARM:{drone_id}:{timestamp}:{status}"
        self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())

    def request_data_stream(self, stream_id, rate):
        """
        Request a specific data stream from Pixhawk.
        """
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            stream_id, rate, 1
        )
