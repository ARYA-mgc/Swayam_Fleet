"""
Swayam Core - MAVLink Multi-Drone Fleet Management with INS Navigation
======================================================================
Author: Swayam Project Team
License: MIT
"""

import time
import math
import json
import heapq
import sqlite3
import threading
import logging
from datetime import datetime
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass, field

# --- Optional MAVLink import (sim-safe) ---
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[WARN] pymavlink not installed — running in simulation mode.")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("swayam")


# ──────────────────────────────────────────────
# 1. INS State — Dead-Reckoning Navigation
# ──────────────────────────────────────────────

@dataclass
class INSState:
    """
    Inertial Navigation System — integrates IMU data to track
    position/velocity without GPS.

    Coordinate system: NED (North-East-Down)
    Body frame: X=forward, Y=right, Z=down
    """
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [N, E, D] meters
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [Vn, Ve, Vd] m/s
    attitude: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [roll, pitch, yaw] rad
    timestamp: float = field(default_factory=time.time)
    gravity: float = 9.80665  # m/s²
    drift_noise: float = 0.001  # small realistic sensor noise

    def rotate_body_to_world(self, accel_body: List[float]) -> List[float]:
        """Rotate acceleration vector from body frame to world (NED) frame."""
        r, p, y = self.attitude
        # Rotation matrix rows (simplified ZYX Euler)
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        ax, ay, az = accel_body
        # Full rotation matrix R = Rz(yaw)*Ry(pitch)*Rx(roll)
        world_x = (cy*cp)*ax + (cy*sp*sr - sy*cr)*ay + (cy*sp*cr + sy*sr)*az
        world_y = (sy*cp)*ax + (sy*sp*sr + cy*cr)*ay + (sy*sp*cr - cy*sr)*az
        world_z = (-sp)*ax + (cp*sr)*ay + (cp*cr)*az
        return [world_x, world_y, world_z]

    def integrate(self, accel_body: List[float], gyro: List[float], dt: float):
        """
        One INS integration step.
        1. Update attitude from gyro
        2. Rotate accel to world frame
        3. Remove gravity
        4. Integrate velocity & position
        """
        if dt <= 0:
            return

        # Update attitude
        self.attitude[0] += gyro[0] * dt  # roll
        self.attitude[1] += gyro[1] * dt  # pitch
        self.attitude[2] += gyro[2] * dt  # yaw

        # Rotate acceleration to world frame
        accel_world = self.rotate_body_to_world(accel_body)

        # Remove gravity from Z (down) component
        accel_world[2] += self.gravity  # NED: gravity is +Z (down)

        # Add tiny sensor drift noise
        import random
        accel_world = [a + random.gauss(0, self.drift_noise) for a in accel_world]

        # Integrate velocity (trapezoidal)
        for i in range(3):
            self.velocity[i] += accel_world[i] * dt

        # Integrate position
        for i in range(3):
            self.position[i] += self.velocity[i] * dt

        self.timestamp = time.time()

    def reset(self):
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]

    def to_dict(self) -> dict:
        return {
            "position_N": round(self.position[0], 4),
            "position_E": round(self.position[1], 4),
            "position_D": round(self.position[2], 4),
            "velocity_N": round(self.velocity[0], 4),
            "velocity_E": round(self.velocity[1], 4),
            "velocity_D": round(self.velocity[2], 4),
            "roll_rad":   round(self.attitude[0], 6),
            "pitch_rad":  round(self.attitude[1], 6),
            "yaw_rad":    round(self.attitude[2], 6),
            "timestamp":  self.timestamp,
        }


# ──────────────────────────────────────────────
# 2. Grid Map + A* Path Planner
# ──────────────────────────────────────────────

class GridMap:
    """
    2D occupancy grid map for path planning.
    Cell size = 1m × 1m. Origin at (0,0).
    0 = free, 1 = obstacle.
    """

    def __init__(self, width: int = 50, height: int = 50):
        self.width = width
        self.height = height
        self.grid = [[0] * width for _ in range(height)]
        self.obstacles: List[Tuple[int, int]] = []

    def add_obstacle(self, x: int, y: int, radius: int = 1):
        """Add a circular obstacle at grid cell (x, y)."""
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.grid[ny][nx] = 1
                    self.obstacles.append((nx, ny))

    def is_free(self, x: int, y: int) -> bool:
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x] == 0
        return False

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A* pathfinding with 8-directional movement and Euclidean heuristic.
        Returns list of (x, y) grid cells from start to goal (inclusive).
        """
        if not self.is_free(*start) or not self.is_free(*goal):
            logger.warning(f"A*: start {start} or goal {goal} is blocked.")
            return []

        def h(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        DIRS = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
        open_set = [(0 + h(start, goal), 0, start, [start])]
        visited = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return path
            for dx, dy in DIRS:
                nx, ny = current[0] + dx, current[1] + dy
                nxt = (nx, ny)
                if nxt not in visited and self.is_free(nx, ny):
                    step = math.hypot(dx, dy)
                    ng = g + step
                    heapq.heappush(open_set, (ng + h(nxt, goal), ng, nxt, path + [nxt]))

        logger.warning(f"A*: No path found from {start} to {goal}")
        return []

    def to_dict(self) -> dict:
        return {"width": self.width, "height": self.height, "obstacles": self.obstacles}


# ──────────────────────────────────────────────
# 3. Drone Agent
# ──────────────────────────────────────────────

class DroneAgent:
    """
    Single drone agent — wraps MAVLink connection, telemetry,
    INS state, and mission execution.
    """

    MODES = {
        "STABILIZE": 0, "ALT_HOLD": 2, "LOITER": 5,
        "RTL": 6, "LAND": 9, "GUIDED": 4, "AUTO": 3,
    }

    def __init__(
        self,
        drone_id: str,
        system_id: int = 1,
        connection_string: str = "udp:127.0.0.1:14550",
        gcs_url: Optional[str] = None,
        simulation: bool = True,
    ):
        self.drone_id = drone_id
        self.system_id = system_id
        self.connection_string = connection_string
        self.gcs_url = gcs_url
        self.simulation = simulation or not MAVLINK_AVAILABLE

        self.ins = INSState()
        self.conn = None
        self.gcs_conn = None
        self._telem_thread: Optional[threading.Thread] = None
        self._relay_thread: Optional[threading.Thread] = None
        self._running = False

        self.battery_pct: float = 100.0
        self.armed: bool = False
        self.mode: str = "STABILIZE"
        self.status_text: str = "INIT"
        self.home_position: Optional[List[float]] = None
        self.current_waypoint: int = 0
        self.mission_active: bool = False

        self.log = logging.getLogger(f"swayam.drone.{drone_id}")
        self.log.info(f"DroneAgent '{drone_id}' (sysid={system_id}) created (sim={self.simulation})")
        if self.gcs_url:
            self.log.info(f"GCS Link enabled: {self.gcs_url}")

    # --- Connection ---
    def connect(self) -> bool:
        self._running = True
        
        # Connect to GCS if specified
        if self.gcs_url and MAVLINK_AVAILABLE:
            try:
                self.log.info(f"Connecting to GCS at {self.gcs_url}...")
                self.gcs_conn = mavutil.mavlink_connection(
                    self.gcs_url,
                    source_system=self.system_id,
                    source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                )
            except Exception as e:
                self.log.error(f"GCS Connection failed: {e}")

        if self.simulation:
            self.log.info("Simulation mode — starting simulated telemetry.")
            self._telem_thread = threading.Thread(target=self._sim_telemetry, daemon=True)
            self._telem_thread.start()
            return True

        try:
            self.log.info(f"Connecting to FCU at {self.connection_string} ...")
            self.conn = mavutil.mavlink_connection(
                self.connection_string,
                source_system=255, # GCS system ID
                source_component=0,
            )
            self.conn.wait_heartbeat(timeout=10)
            self.log.info(f"Heartbeat received from sysid={self.conn.target_system}")
            
            # Start Telemetry Loop
            self._telem_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
            self._telem_thread.start()
            
            # Start Relay Loop if GCS is connected
            if self.gcs_conn:
                self._relay_thread = threading.Thread(target=self._relay_loop, daemon=True)
                self._relay_thread.start()
                
            return True
        except Exception as e:
            self.log.error(f"FCU Connection failed: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._telem_thread:
            self._telem_thread.join(timeout=1)
        if self._relay_thread:
            self._relay_thread.join(timeout=1)
        if self.conn:
            self.conn.close()
        if self.gcs_conn:
            self.gcs_conn.close()
        self.log.info("Disconnected.")

    # --- MAVLink Commands ---
    def arm(self) -> bool:
        self.log.info("Arming motors...")
        if self.simulation:
            self.armed = True
            return True
        self.conn.arducopter_arm()
        self.conn.motors_armed_wait()
        self.armed = True
        return True

    def disarm(self) -> bool:
        self.log.info("Disarming motors...")
        if self.simulation:
            self.armed = False
            return True
        self.conn.arducopter_disarm()
        self.armed = False
        return True

    def set_mode(self, mode_name: str) -> bool:
        self.mode = mode_name
        if self.simulation:
            return True
        mode_id = self.MODES.get(mode_name.upper())
        if mode_id is None:
            self.log.warning(f"Unknown mode: {mode_name}")
            return False
        self.conn.set_mode(mode_id)
        return True

    def send_ned_setpoint(self, north: float, east: float, down: float, yaw: float = 0.0):
        """
        Send SET_POSITION_TARGET_LOCAL_NED — GPS-independent local frame.
        """
        if self.simulation:
            # Simulate movement towards setpoint
            self.ins.position[0] += (north - self.ins.position[0]) * 0.1
            self.ins.position[1] += (east - self.ins.position[1]) * 0.1
            self.ins.position[2] = down
            return

        self.conn.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,   # type_mask: use position
            north, east, down,    # x, y, z
            0, 0, 0,              # velocity
            0, 0, 0,              # acceleration
            yaw, 0,               # yaw, yaw_rate
        )

    def takeoff(self, altitude: float = 10.0):
        self.log.info(f"Taking off to {altitude}m AGL...")
        self.set_mode("GUIDED")
        self.arm()
        if not self.simulation:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude,
            )
        time.sleep(2)
        self.ins.position[2] = -altitude  # NED down is negative for altitude

    def land(self):
        self.log.info("Landing...")
        self.set_mode("LAND")
        if not self.simulation:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
        time.sleep(3)
        self.ins.position[2] = 0.0
        self.disarm()

    # --- Telemetry Loops ---
    def _telemetry_loop(self):
        """Real MAVLink telemetry — runs in background thread."""
        last_t = time.time()
        while self._running:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                if not msg:
                    continue
                
                # Forward to GCS if relay is active
                if self.gcs_conn:
                    self.gcs_conn.mav.send(msg)

                now = time.time()
                dt = now - last_t
                last_t = now
                mtype = msg.get_type()

                if mtype == "SCALED_IMU2":
                    accel = [msg.xacc / 1000.0, msg.yacc / 1000.0, msg.zacc / 1000.0]
                    gyro  = [msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0]
                    self.ins.integrate(accel, gyro, dt)

                elif mtype == "SYS_STATUS":
                    self.battery_pct = msg.battery_remaining

                elif mtype == "STATUSTEXT":
                    self.status_text = msg.text
                    self.log.info(f"FC: {msg.text}")

            except Exception as e:
                self.log.debug(f"Telemetry error: {e}")

    def _relay_loop(self):
        """Forward commands from GCS to FCU."""
        while self._running:
            try:
                msg = self.gcs_conn.recv_match(blocking=True, timeout=0.1)
                if not msg:
                    continue
                
                # Forward to FCU
                self.conn.mav.send(msg)
                self.log.debug(f"Relayed GCS -> FCU: {msg.get_type()}")
            except Exception as e:
                self.log.debug(f"Relay error: {e}")

    def _sim_telemetry(self):
        """Simulated IMU + MAVLink telemetry for Mission Planner."""
        import random
        last_t = time.time()
        last_hb = 0
        tick = 0
        
        while self._running:
            time.sleep(0.05)
            now = time.time()
            dt = now - last_t
            last_t = now
            tick += 1

            # 1. Physical Simulation
            accel = [
                0.05 * math.sin(tick * 0.1),
                0.01 * math.cos(tick * 0.07),
                -9.80665 + random.gauss(0, 0.02)
            ]
            gyro = [
                random.gauss(0, 0.002),
                random.gauss(0, 0.002),
                0.01 * math.sin(tick * 0.05),
            ]
            self.ins.integrate(accel, gyro, dt)
            self.battery_pct = max(0, self.battery_pct - 0.0001)

            # 2. MAVLink Output (to Mission Planner)
            if self.gcs_conn:
                # Heartbeat (1Hz)
                if now - last_hb > 1.0:
                    self.gcs_conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED | 
                        (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0),
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE if self.armed else mavutil.mavlink.MAV_STATE_STANDBY
                    )
                    last_hb = now

                # Position (5Hz)
                if tick % 4 == 0:
                    # LOCAL_POSITION_NED
                    self.gcs_conn.mav.local_position_ned_send(
                        int(now * 1000) & 0xFFFFFFFF,
                        self.ins.position[0], self.ins.position[1], self.ins.position[2],
                        self.ins.velocity[0], self.ins.velocity[1], self.ins.velocity[2]
                    )
                    # GLOBAL_POSITION_INT (placeholder GPS for Mission Planner map)
                    # Use a fixed origin (e.g. 12.9716, 77.5946 - Bangalore)
                    lat, lon = 12.9716, 77.5946
                    # Approx 1m = 0.00001 deg
                    plat = int((lat + self.ins.position[0] * 0.000009) * 1e7)
                    plon = int((lon + self.ins.position[1] * 0.000009) * 1e7)
                    self.gcs_conn.mav.global_position_int_send(
                        int(now * 1000) & 0xFFFFFFFF,
                        plat, plon, int(-self.ins.position[2] * 1000), # alt
                        int(-self.ins.position[2] * 1000), # relative_alt
                        int(self.ins.velocity[0] * 100), int(self.ins.velocity[1] * 100), int(self.ins.velocity[2] * 100),
                        int(self.ins.attitude[2] * 100) # hdg
                    )

                # Attitude (10Hz)
                if tick % 2 == 0:
                    self.gcs_conn.mav.attitude_send(
                        int(now * 1000) & 0xFFFFFFFF,
                        self.ins.attitude[0], self.ins.attitude[1], self.ins.attitude[2],
                        0, 0, 0 # rates
                    )

                # Battery (2Hz)
                if tick % 10 == 0:
                    self.gcs_conn.mav.sys_status_send(
                        0, 0, 0, 500, # load, sensors, etc
                        11100, # 11.1V
                        int(self.battery_pct * 10), # remaining
                        0, 0, 0, 0, 0, 0, 0
                    )

    # --- Status ---
    def status(self) -> dict:
        return {
            "drone_id":    self.drone_id,
            "system_id":   self.system_id,
            "armed":       self.armed,
            "mode":        self.mode,
            "battery_pct": round(self.battery_pct, 1),
            "status_text": self.status_text,
            "mission_active": self.mission_active,
            "ins":         self.ins.to_dict(),
        }


# ──────────────────────────────────────────────
# 4. Flight Database
# ──────────────────────────────────────────────

class FlightDatabase:
    """
    SQLite-backed persistent store for:
    - flight_logs: timestamped events per drone
    - ins_telemetry: position/velocity snapshots
    - missions: mission records with paths
    """

    def __init__(self, db_path: str = "swayam_flights.db"):
        self.db_path = db_path
        self._local = threading.local()
        self._init_schema()

    def _conn(self) -> sqlite3.Connection:
        if not hasattr(self._local, "conn") or self._local.conn is None:
            self._local.conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self._local.conn.row_factory = sqlite3.Row
        return self._local.conn

    def _init_schema(self):
        c = self._conn()
        c.executescript("""
        CREATE TABLE IF NOT EXISTS flight_logs (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            drone_id    TEXT NOT NULL,
            timestamp   REAL NOT NULL,
            level       TEXT DEFAULT 'INFO',
            event       TEXT NOT NULL,
            details     TEXT
        );
        CREATE TABLE IF NOT EXISTS ins_telemetry (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            drone_id    TEXT NOT NULL,
            timestamp   REAL NOT NULL,
            pos_n       REAL, pos_e REAL, pos_d REAL,
            vel_n       REAL, vel_e REAL, vel_d REAL,
            roll        REAL, pitch REAL, yaw REAL
        );
        CREATE TABLE IF NOT EXISTS missions (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            drone_id    TEXT NOT NULL,
            start_time  REAL NOT NULL,
            end_time    REAL,
            status      TEXT DEFAULT 'PLANNED',
            path_json   TEXT,
            notes       TEXT
        );
        CREATE INDEX IF NOT EXISTS idx_logs_drone ON flight_logs(drone_id);
        CREATE INDEX IF NOT EXISTS idx_telem_drone ON ins_telemetry(drone_id);
        """)
        c.commit()

    def log_event(self, drone_id: str, event: str, details: str = "", level: str = "INFO"):
        c = self._conn()
        c.execute(
            "INSERT INTO flight_logs (drone_id, timestamp, level, event, details) VALUES (?,?,?,?,?)",
            (drone_id, time.time(), level, event, details)
        )
        c.commit()

    def log_ins(self, drone_id: str, ins: INSState):
        c = self._conn()
        c.execute(
            """INSERT INTO ins_telemetry
               (drone_id, timestamp, pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, roll, pitch, yaw)
               VALUES (?,?,?,?,?,?,?,?,?,?,?)""",
            (drone_id, ins.timestamp,
             *ins.position, *ins.velocity, *ins.attitude)
        )
        c.commit()

    def create_mission(self, drone_id: str, path: List[Tuple[int, int]], notes: str = "") -> int:
        c = self._conn()
        cur = c.execute(
            "INSERT INTO missions (drone_id, start_time, status, path_json, notes) VALUES (?,?,?,?,?)",
            (drone_id, time.time(), "ACTIVE", json.dumps(path), notes)
        )
        c.commit()
        return cur.lastrowid

    def complete_mission(self, mission_id: int, status: str = "COMPLETED"):
        c = self._conn()
        c.execute("UPDATE missions SET end_time=?, status=? WHERE id=?",
                  (time.time(), status, mission_id))
        c.commit()

    def get_recent_logs(self, drone_id: Optional[str] = None, limit: int = 100) -> List[dict]:
        c = self._conn()
        if drone_id:
            rows = c.execute(
                "SELECT * FROM flight_logs WHERE drone_id=? ORDER BY timestamp DESC LIMIT ?",
                (drone_id, limit)
            ).fetchall()
        else:
            rows = c.execute(
                "SELECT * FROM flight_logs ORDER BY timestamp DESC LIMIT ?", (limit,)
            ).fetchall()
        return [dict(r) for r in rows]

    def get_ins_history(self, drone_id: str, limit: int = 200) -> List[dict]:
        c = self._conn()
        rows = c.execute(
            "SELECT * FROM ins_telemetry WHERE drone_id=? ORDER BY timestamp DESC LIMIT ?",
            (drone_id, limit)
        ).fetchall()
        return [dict(r) for r in rows]

    def export_json(self, path: str = "swayam_export.json"):
        data = {
            "exported_at": datetime.utcnow().isoformat(),
            "flight_logs": self.get_recent_logs(limit=9999),
            "ins_telemetry": [],
        }
        c = self._conn()
        rows = c.execute("SELECT * FROM ins_telemetry ORDER BY timestamp").fetchall()
        data["ins_telemetry"] = [dict(r) for r in rows]
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        logger.info(f"Exported DB to {path}")


# ──────────────────────────────────────────────
# 5. Swayam Fleet Manager
# ──────────────────────────────────────────────

class SwayamFleet:
    """
    Fleet manager — coordinates multiple DroneAgents,
    plans paths via A*, logs everything to FlightDatabase.
    """

    def __init__(self, db_path: str = "swayam_flights.db", map_size: int = 50):
        self.drones: Dict[str, DroneAgent] = {}
        self.db = FlightDatabase(db_path)
        self.grid = GridMap(map_size, map_size)
        self.log = logging.getLogger("swayam.fleet")

    def add_drone(
        self,
        drone_id: str,
        system_id: int = 1,
        connection_string: str = "udp:127.0.0.1:14550",
        gcs_url: Optional[str] = None,
        simulation: bool = True,
    ) -> DroneAgent:
        drone = DroneAgent(drone_id, system_id, connection_string, gcs_url, simulation)
        self.drones[drone_id] = drone
        self.db.log_event(drone_id, "DRONE_REGISTERED", f"conn={connection_string} gcs={gcs_url} sim={simulation}")
        return drone

    def add_obstacle(self, x: int, y: int, radius: int = 1):
        self.grid.add_obstacle(x, y, radius)
        self.log.info(f"Obstacle added at ({x},{y}) r={radius}")

    def connect_all(self) -> Dict[str, bool]:
        results = {}
        for did, drone in self.drones.items():
            ok = drone.connect()
            results[did] = ok
            self.db.log_event(did, "CONNECTED" if ok else "CONNECT_FAILED")
        return results

    def disconnect_all(self):
        for drone in self.drones.values():
            drone.disconnect()

    def plan_path(
        self,
        drone_id: str,
        goal_n: float,
        goal_e: float,
    ) -> List[Tuple[int, int]]:
        """Plan an A* path from drone's current INS position to goal."""
        drone = self.drones[drone_id]
        sx = int(drone.ins.position[0]) + self.grid.width // 2
        sy = int(drone.ins.position[1]) + self.grid.height // 2
        gx = int(goal_n) + self.grid.width // 2
        gy = int(goal_e) + self.grid.height // 2
        sx, sy = max(0, min(sx, self.grid.width-1)), max(0, min(sy, self.grid.height-1))
        gx, gy = max(0, min(gx, self.grid.width-1)), max(0, min(gy, self.grid.height-1))
        path = self.grid.astar((sx, sy), (gx, gy))
        self.log.info(f"[{drone_id}] Path planned: {len(path)} waypoints")
        return path

    def execute_mission(
        self,
        drone_id: str,
        goal_n: float,
        goal_e: float,
        altitude: float = 10.0,
        speed: float = 2.0,
    ):
        """Full mission: arm → takeoff → fly path → land → disarm."""
        drone = self.drones[drone_id]
        path = self.plan_path(drone_id, goal_n, goal_e)
        if not path:
            self.db.log_event(drone_id, "MISSION_ABORT", "No path found", "ERROR")
            return

        mission_id = self.db.create_mission(drone_id, path, f"goal=({goal_n},{goal_e})")
        self.db.log_event(drone_id, "MISSION_START", f"goal=({goal_n},{goal_e}) wpts={len(path)}")

        try:
            drone.takeoff(altitude)
            self.db.log_event(drone_id, "TAKEOFF", f"alt={altitude}m")
            drone.mission_active = True

            for i, (wx, wy) in enumerate(path):
                # Convert grid cell back to NED
                north = wx - self.grid.width // 2
                east  = wy - self.grid.height // 2
                drone.send_ned_setpoint(north, east, -altitude)
                drone.current_waypoint = i
                self.db.log_ins(drone_id, drone.ins)
                self.db.log_event(drone_id, "WAYPOINT", f"wp={i} N={north} E={east}")
                time.sleep(1.0 / speed)

            drone.land()
            self.db.log_event(drone_id, "LANDED")
            drone.mission_active = False
            self.db.complete_mission(mission_id, "COMPLETED")

        except Exception as e:
            self.db.log_event(drone_id, "MISSION_ERROR", str(e), "ERROR")
            self.db.complete_mission(mission_id, "FAILED")
            raise

    def broadcast_destination(self, goal_n: float, goal_e: float, altitude: float = 10.0):
        """Send all drones to the same destination in parallel."""
        threads = []
        for did in self.drones:
            t = threading.Thread(
                target=self.execute_mission,
                args=(did, goal_n, goal_e, altitude),
                daemon=True,
            )
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        self.log.info("Broadcast mission complete.")

    def fleet_status(self) -> List[dict]:
        return [d.status() for d in self.drones.values()]

    def emergency_land_all(self):
        self.log.warning("EMERGENCY LAND — ALL DRONES")
        for did, drone in self.drones.items():
            drone.set_mode("LAND")
            self.db.log_event(did, "EMERGENCY_LAND", level="ERROR")


# ──────────────────────────────────────────────
# Entry Point — Demo / Simulation
# ──────────────────────────────────────────────

if __name__ == "__main__":
    fleet = SwayamFleet(db_path="swayam_flights.db")

    # Add obstacles
    fleet.add_obstacle(10, 10, radius=2)
    fleet.add_obstacle(20, 15, radius=3)
    fleet.add_obstacle(30, 25, radius=2)

    # Register drones (simulation=True — no hardware needed)
    fleet.add_drone("ALPHA", system_id=1, simulation=True)
    fleet.add_drone("BETA",  system_id=2, simulation=True)
    fleet.add_drone("GAMMA", system_id=3, simulation=True)

    # Connect
    fleet.connect_all()

    # Run individual missions
    import threading
    missions = [
        ("ALPHA", 15.0, 12.0, 10.0),
        ("BETA",  -8.0, 18.0, 15.0),
        ("GAMMA",  5.0, -5.0, 12.0),
    ]

    threads = []
    for drone_id, gn, ge, alt in missions:
        t = threading.Thread(target=fleet.execute_mission, args=(drone_id, gn, ge, alt))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # Export logs
    fleet.db.export_json("logs/swayam_export.json")

    print("\n=== Fleet Status ===")
    for s in fleet.fleet_status():
        print(json.dumps(s, indent=2))

    fleet.disconnect_all()
    print("\nSwayam simulation complete.")
