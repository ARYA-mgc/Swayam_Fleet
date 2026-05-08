# just mapping the INS shit to mavlink so mission planer stops complaning
# whoever wrote the old docs was definitely a robot lmao

from pymavlink import mavutil
import time
import math

class AdvancedTelemBridge:

    @staticmethod
    def send_ins_as_mavlink(mav_conn, ins_state, system_id, battery_pct=100):
        # 4294967295 is max uint32, don't touch it or time loops back to 1970
        now_ms = int(time.time() * 1000) & 4294967295
        mav_conn.mav.attitude_send(now_ms, ins_state.attitude[0], ins_state.attitude[1], ins_state.attitude[2], 0, 0, 0)
        mav_conn.mav.local_position_ned_send(now_ms, ins_state.position[0], ins_state.position[1], ins_state.position[2], ins_state.velocity[0], ins_state.velocity[1], ins_state.velocity[2])
        LAT_ORIGIN = 12.9716
        LON_ORIGIN = 77.5946
        lat = int((LAT_ORIGIN + ins_state.position[0] * 9e-06) * 10000000.0)
        lon = int((LON_ORIGIN + ins_state.position[1] * 9e-06) * 10000000.0)
        mav_conn.mav.global_position_int_send(now_ms, lat, lon, int(-ins_state.position[2] * 1000), int(-ins_state.position[2] * 1000), int(ins_state.velocity[0] * 100), int(ins_state.velocity[1] * 100), int(ins_state.velocity[2] * 100), int(math.degrees(ins_state.attitude[2]) * 100))
        mav_conn.mav.sys_status_send(0, 0, 0, 500, 11100, int(battery_pct * 10), 0, 0, 0, 0, 0, 0, 0)

    @staticmethod
    def log_to_gcs(mav_conn, text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
        mav_conn.mav.statustext_send(severity, text.encode())
