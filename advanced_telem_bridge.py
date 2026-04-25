"""
Swayam Advanced Telemetry Bridge
================================
Maps INSState (NED position, velocity, attitude) to standard MAVLink messages
for high-fidelity visualization in Mission Planner.
"""

from pymavlink import mavutil
import time
import math

class AdvancedTelemBridge:
    @staticmethod
    def send_ins_as_mavlink(mav_conn, ins_state, system_id, battery_pct=100):
        """
        Takes an INSState object and sends multiple MAVLink messages.
        """
        now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        
        # 1. ATTITUDE
        mav_conn.mav.attitude_send(
            now_ms,
            ins_state.attitude[0], # roll
            ins_state.attitude[1], # pitch
            ins_state.attitude[2], # yaw
            0, 0, 0 # body rates
        )
        
        # 2. LOCAL_POSITION_NED
        mav_conn.mav.local_position_ned_send(
            now_ms,
            ins_state.position[0], # x (N)
            ins_state.position[1], # y (E)
            ins_state.position[2], # z (D)
            ins_state.velocity[0], # vx
            ins_state.velocity[1], # vy
            ins_state.velocity[2]  # vz
        )
        
        # 3. GLOBAL_POSITION_INT (Requires lat/lon conversion)
        # Mocking a global origin for Mission Planner map
        LAT_ORIGIN = 12.9716
        LON_ORIGIN = 77.5946
        lat = int((LAT_ORIGIN + ins_state.position[0] * 0.000009) * 1e7)
        lon = int((LON_ORIGIN + ins_state.position[1] * 0.000009) * 1e7)
        
        mav_conn.mav.global_position_int_send(
            now_ms,
            lat, lon, 
            int(-ins_state.position[2] * 1000), # alt (mm)
            int(-ins_state.position[2] * 1000), # relative_alt (mm)
            int(ins_state.velocity[0] * 100),   # vx (cm/s)
            int(ins_state.velocity[1] * 100),   # vy (cm/s)
            int(ins_state.velocity[2] * 100),   # vz (cm/s)
            int(math.degrees(ins_state.attitude[2]) * 100) # hdg (cdeg)
        )
        
        # 4. SYS_STATUS (Battery)
        mav_conn.mav.sys_status_send(
            0, 0, 0, 500,
            11100, # 11.1V
            int(battery_pct * 10), # remaining (%)
            0, 0, 0, 0, 0, 0, 0
        )

    @staticmethod
    def log_to_gcs(mav_conn, text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
        """Sends a STATUSTEXT message."""
        mav_conn.mav.statustext_send(severity, text.encode())
