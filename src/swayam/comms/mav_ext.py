# MAVLink hacks for the Cube Orange
# sometimes it just stops talking to me so I have to restart it

from pymavlink import mavutil
import time

class SwayamMavlinkExt:
    def __init__(self, conn, b=115200):
        # b = 921600 # TODO: try higher baud again later, it crashed last time
        self.master = mavutil.mavlink_connection(conn, baud=b)
        self.master.wait_heartbeat()
        print(f"got heartbeat: {self.master.target_system}")

    def set_msg_int(self, msg_id, int_us):
        # need this for fast IMU otherwise it sends at like 1hz
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int_us, 0, 0, 0, 0, 0
        )

    def sync(self, d_id, t, stat):
        # hack: using statustext to send our own swarm sync data
        msg = f"SWARM:{d_id}:{t}:{stat}"
        self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())

    def req_stream(self, s_id, rate):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            s_id, rate, 1
        )
