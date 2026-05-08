# the big brain of the operation. glues all the random parts together
# if it crashes, it's probably mavlink's fault not mine
# - ARYA-mgc / MIT (whatever that means)

import time
import math
import json
import heapq
import sqlite3
import asyncio
import logging
import random
import threading  # not used directly but some old code needed it, keep for now
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass, field
from src.swayam.core.navcore.api import ESKFState, ESKFPublisher
from src.swayam.control.logic import AutonomousSwarmLogic
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print('[WARN] pymavlink not installed — running in simulation mode.')
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(name)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
logger = logging.getLogger('swayam')

class ESKFStateInput:

    def __init__(self, publisher: ESKFPublisher):
        self._pub = publisher
        self._last_valid_pos = [0.0, 0.0, 0.0]
        self._velocity_decay = 1.0
        self._health_state = 'HEALTHY'
        self._bad_samples = 0
        self._good_samples = 0
        self._last_t = time.time()

    @property
    def position(self) -> List[float]:
        self.confidence_weight
        st = self._pub.latest()
        if self._health_state == 'FAULT':
            return self._last_valid_pos
        self._last_valid_pos = [st.px, st.py, st.pz]
        return self._last_valid_pos

    @property
    def velocity(self) -> List[float]:
        st = self._pub.latest()
        now = time.time()
        dt = max(0.001, now - self._last_t)
        self._last_t = now
        if self._health_state == 'FAULT':
            tau = 0.5
            self._velocity_decay *= math.exp(-dt / tau)
            return [st.vx * self._velocity_decay, st.vy * self._velocity_decay, st.vz * self._velocity_decay]
        self._velocity_decay = 1.0
        return [st.vx, st.vy, st.vz]

    @property
    def attitude(self) -> List[float]:
        st = self._pub.latest()
        qw, qx, qy, qz = (st.qw, st.qx, st.qy, st.qz)
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = math.asin(2.0 * (qw * qy - qz * qx))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return [roll, pitch, yaw]

    @property
    def timestamp(self) -> float:
        return self._pub.latest().t

    @property
    def confidence_weight(self) -> float:
        st = self._pub.latest()
        if st.health in ['FAULT', 'WARNING']:
            self._bad_samples += 1
            self._good_samples = 0
        else:
            self._good_samples += 1
            if self._good_samples >= 5:
                self._bad_samples = 0
                self._health_state = 'HEALTHY'
        if self._health_state == 'HEALTHY' and self._bad_samples >= 2:
            self._health_state = 'WARNING'
        elif self._health_state == 'WARNING' and self._bad_samples >= 5:
            self._health_state = 'FAULT'
        if self._health_state == 'FAULT':
            return 0.0
        elif self._health_state == 'WARNING':
            return 0.5
        return 1.0

    def to_dict(self) -> dict:
        st = self._pub.latest()
        return {'position_N': round(st.px, 4), 'position_E': round(st.py, 4), 'position_D': round(st.pz, 4), 'velocity_N': round(st.vx, 4), 'velocity_E': round(st.vy, 4), 'velocity_D': round(st.vz, 4), 'roll_rad': round(self.attitude[0], 6), 'pitch_rad': round(self.attitude[1], 6), 'yaw_rad': round(self.attitude[2], 6), 'health': st.health, 'timestamp': st.t}

    def simulate_update(self, dt: float, v_cmd: List[float], noise_std: float=0.0, wind: Tuple[float, float, float]=(0.0, 0.0, 0.0)):
        # OK SO THIS IS THE MOST IMPORTANT PART OF THE SIMULATION
        # IF YOU MESS WITH THIS THE DRONES WILL FLY BACKWARDS
        # 
        # Basically we need to take the velocity command from the PID controller
        # and pretend it went through the flight controller. 
        # The FCU has a slight delay which we model as a first-order low pass filter
        # with tau = 0.05. I measured this by swinging the drone around in my living room
        #
        # Then we also have to add wind and drag because otherwise the PID controller
        # thinks it's flying in a vacuum and it overshoots like crazy.
        # Drag coefficient is 0.5 because it made the graphs look nice.
        # Do NOT change tau unless you recalibrate the entire PID loop!!!
        st = self._pub.latest()
        tau = 0.05
        a_fcu_n = (v_cmd[0] - st.vx) / tau
        a_fcu_e = (v_cmd[1] - st.vy) / tau
        a_fcu_d = (v_cmd[2] - st.vz) / tau
        wind_n, wind_e, wind_d = wind
        airspeed_n = st.vx - wind_n
        airspeed_e = st.vy - wind_e
        airspeed_d = st.vz - wind_d
        cd = 0.5
        a_drag_n = -cd * airspeed_n
        a_drag_e = -cd * airspeed_e
        a_drag_d = -cd * airspeed_d
        a_tot_n = a_fcu_n + a_drag_n
        a_tot_e = a_fcu_e + a_drag_e
        a_tot_d = a_fcu_d + a_drag_d
        nvx = st.vx + a_tot_n * dt
        nvy = st.vy + a_tot_e * dt
        nvz = st.vz + a_tot_d * dt
        nx = st.px + nvx * dt
        ny = st.py + nvy * dt
        nz = st.pz + nvz * dt
        if noise_std > 0.0:
            if random.random() < 0.02:
                nx += random.gauss(0, noise_std * 10.0)
                ny += random.gauss(0, noise_std * 10.0)
            else:
                nx += random.gauss(0, noise_std)
                ny += random.gauss(0, noise_std)
        new_state = ESKFState(t=time.time(), px=nx, py=ny, pz=nz, vx=nvx, vy=nvy, vz=nvz, qw=st.qw, qx=st.qx, qy=st.qy, qz=st.qz, cov_trace=st.cov_trace, health=st.health)
        self._pub.publish(new_state)

class GridMap:

    def __init__(self, width: int=50, height: int=50):
        self.width = width
        self.height = height
        self.grid = [[0] * width for _ in range(height)]
        self.obstacles: List[Tuple[int, int]] = []

    def add_obstacle(self, x: int, y: int, radius: int=1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = (x + dx, y + dy)
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.grid[ny][nx] = 1
                    self.obstacles.append((nx, ny))

    def is_free(self, x: int, y: int) -> bool:
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x] == 0
        return False

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        if not self.is_free(*start) or not self.is_free(*goal):
            logger.warning(f'A*: start {start} or goal {goal} is blocked.')
            return []

        def h(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])
        DIRS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
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
                nx, ny = (current[0] + dx, current[1] + dy)
                nxt = (nx, ny)
                if nxt not in visited and self.is_free(nx, ny):
                    step = math.hypot(dx, dy)
                    ng = g + step
                    heapq.heappush(open_set, (ng + h(nxt, goal), ng, nxt, path + [nxt]))
        logger.warning(f'A*: No path found from {start} to {goal}')
        return []

    def to_dict(self) -> dict:
        return {'width': self.width, 'height': self.height, 'obstacles': self.obstacles}

class WindEnvironment:
    # wind model stolen from some paper i can't find anymore
    # works well enough for 5m/s conditions
    _t_last = 0.0
    _base_wind_n = 2.0
    _base_wind_e = 1.0
    _gust_time = 0.0
    _gust_vector = (0.0, 0.0)

    @classmethod
    def get_wind(cls, current_time: float) -> Tuple[float, float, float]:
        if cls._t_last == 0.0:
            cls._t_last = current_time
            return (cls._base_wind_n, cls._base_wind_e, 0.0)
        dt = current_time - cls._t_last
        cls._t_last = current_time
        cls._base_wind_n += random.gauss(0, 0.2 * dt)
        cls._base_wind_e += random.gauss(0, 0.2 * dt)
        mag = math.hypot(cls._base_wind_n, cls._base_wind_e)
        if mag > 6.0:
            cls._base_wind_n = cls._base_wind_n / mag * 6.0
            cls._base_wind_e = cls._base_wind_e / mag * 6.0
        if cls._gust_time > 0:
            cls._gust_time -= dt
            w_n = cls._base_wind_n + cls._gust_vector[0]
            w_e = cls._base_wind_e + cls._gust_vector[1]
        else:
            if random.random() < 0.01 * dt:
                cls._gust_time = random.uniform(1.0, 2.0)
                cls._gust_vector = (cls._base_wind_n * random.uniform(1.5, 2.5), cls._base_wind_e * random.uniform(1.5, 2.5))
            w_n = cls._base_wind_n
            w_e = cls._base_wind_e
        return (w_n, w_e, 0.0)

class DroneAgent:
    MODES = {'STABILIZE': 0, 'ALT_HOLD': 2, 'LOITER': 5, 'RTL': 6, 'LAND': 9, 'GUIDED': 4, 'AUTO': 3}

    def __init__(self, drone_id: str, system_id: int=1, connection_string: str='udp:127.0.0.1:14550', gcs_url: Optional[str]=None, simulation: bool=True):
        self.drone_id = drone_id
        self.system_id = system_id
        self.connection_string = connection_string
        self.gcs_url = gcs_url
        self.simulation = simulation or not MAVLINK_AVAILABLE
        self.eskf_pub = ESKFPublisher()
        self.ins = ESKFStateInput(self.eskf_pub)
        self.swarm_logic = AutonomousSwarmLogic(my_id=drone_id)
        self.conn = None
        self.gcs_conn = None
        self._tasks: List[asyncio.Task] = []
        self._running = False
        self.battery_pct: float = 100.0
        self.armed: bool = False
        self.mode: str = 'STABILIZE'
        self.status_text: str = 'INIT'
        self.home_position: Optional[List[float]] = None
        self.current_waypoint: int = 0
        self.mission_active: bool = False
        self.log = logging.getLogger(f'swayam.drone.{drone_id}')
        self.log.info(f"DroneAgent '{drone_id}' (sysid={system_id}) created (sim={self.simulation})")
        if self.gcs_url:
            self.log.info(f'GCS Link enabled: {self.gcs_url}')

    async def connect(self) -> bool:
        self._running = True
        if self.gcs_url and MAVLINK_AVAILABLE:
            try:
                self.log.info(f'Connecting to GCS at {self.gcs_url}...')
                self.gcs_conn = mavutil.mavlink_connection(self.gcs_url, source_system=self.system_id, source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
            except Exception as e:
                self.log.error(f'GCS Connection failed: {e}')
        if self.simulation:
            self.log.info('Simulation mode — starting simulated telemetry.')
            self._tasks.append(asyncio.create_task(self._sim_telemetry()))
            return True
        try:
            self.log.info(f'Connecting to FCU at {self.connection_string} ...')
            self.conn = mavutil.mavlink_connection(self.connection_string, source_system=255, source_component=0)
            self.log.info('Waiting for heartbeat...')
            for _ in range(100):
                if self.conn.recv_match(type='HEARTBEAT', blocking=False):
                    break
                await asyncio.sleep(0.1)
            self.log.info(f'Heartbeat received from sysid={self.conn.target_system}')
            self._tasks.append(asyncio.create_task(self._telemetry_loop()))
            if self.gcs_conn:
                self._tasks.append(asyncio.create_task(self._relay_loop()))
            return True
        except Exception as e:
            self.log.error(f'FCU Connection failed: {e}')
            return False

    async def disconnect(self):
        self._running = False
        for t in self._tasks:
            t.cancel()
        if self.conn:
            self.conn.close()
        if self.gcs_conn:
            self.gcs_conn.close()
        self.log.info('Disconnected.')

    def arm(self) -> bool:
        self.log.info('Arming motors...')
        if self.simulation:
            self.armed = True
            return True
        self.conn.arducopter_arm()
        self.conn.motors_armed_wait()
        self.armed = True
        return True

    def disarm(self) -> bool:
        self.log.info('Disarming motors...')
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
            self.log.warning(f'Unknown mode: {mode_name}')
            return False
        self.conn.set_mode(mode_id)
        return True

    def send_ned_setpoint(self, north: float, east: float, down: float, yaw: float=0.0):
        if self.simulation:
            self.mission_target = [north, east, down]
            return
        self.conn.mav.set_position_target_local_ned_send(int(time.time() * 1000) & 4294967295, self.conn.target_system, self.conn.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 4088, north, east, down, 0, 0, 0, 0, 0, 0, yaw, 0)

    async def takeoff(self, altitude: float=10.0):
        self.log.info(f'Taking off to {altitude}m AGL...')
        self.set_mode('GUIDED')
        self.arm()
        if not self.simulation:
            self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
        await asyncio.sleep(2)

    async def land(self):
        self.log.info('Landing...')
        self.set_mode('LAND')
        if not self.simulation:
            self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        await asyncio.sleep(3)
        self.disarm()

    async def _telemetry_loop(self):
        last_msg_time = time.time()
        grace_window_active = False
        last_escalation_state = None
        missed_cycles = 0
        cycle_time = 0.05
        while self._running:
            try:
                now = time.time()
                elapsed = now - last_msg_time
                expected_misses = int(elapsed / cycle_time)
                if expected_misses > 2:
                    missed_cycles += 1
                else:
                    missed_cycles = 0
                if missed_cycles > 2:
                    if elapsed > 15.0 and last_escalation_state != 'RTL':
                        self.log.error('Telemetry watchdog [ERR_TIMEOUT]: 15s timeout. Persistent failure. Triggering RTL.')
                        self.set_mode('RTL')
                        last_escalation_state = 'RTL'
                        last_msg_time = now
                    elif elapsed > 8.0 and self.mode != 'GUIDED' and (last_escalation_state != 'GUIDED'):
                        self.log.warning('Telemetry watchdog [WARN_HEARTBEAT_LOSS]: 8s timeout. Degraded mode.')
                        self.set_mode('GUIDED')
                        last_escalation_state = 'GUIDED'
                    elif elapsed > 3.0 and (not grace_window_active):
                        self.log.warning('Telemetry watchdog [WARN_SOCKET]: 3s timeout. Reconnecting socket.')
                        grace_window_active = True
                    elif elapsed > 1.0 and (not grace_window_active) and (last_escalation_state != 'RETRY'):
                        self.log.warning('Telemetry watchdog [WARN_DELAY]: 1s timeout. Retrying.')
                        last_escalation_state = 'RETRY'
                msg = self.conn.recv_match(blocking=False)
                if msg:
                    last_msg_time = time.time()
                    grace_window_active = False
                    last_escalation_state = None
                    missed_cycles = 0
                if not msg:
                    await asyncio.sleep(cycle_time)
                    continue
                last_msg_time = time.time()
                if self.gcs_conn:
                    self.gcs_conn.mav.send(msg)
                mtype = msg.get_type()
                if mtype == 'SYS_STATUS':
                    self.battery_pct = msg.battery_remaining
                elif mtype == 'STATUSTEXT':
                    self.status_text = msg.text
                    self.log.info(f'FC: {msg.text}')
            except Exception as e:
                self.log.debug(f'Telemetry error: {e}')
                await asyncio.sleep(0.1)

    async def _relay_loop(self):
        while self._running:
            try:
                msg = self.gcs_conn.recv_match(blocking=False)
                if not msg:
                    await asyncio.sleep(0.02)
                    continue
                self.conn.mav.send(msg)
                self.log.debug(f'Relayed GCS -> FCU: {msg.get_type()}')
            except Exception as e:
                self.log.debug(f'Relay error: {e}')
                await asyncio.sleep(0.1)

    async def _sim_telemetry(self, packet_drop_prob: float=0.0, delay_ms: float=0.0):
        last_t = time.time()
        last_hb = 0
        tick = 0
        target_pos = [0, 0, 0]
        while self._running:
            await asyncio.sleep(0.05)
            now = time.time()
            dt = now - last_t
            last_t = now
            tick += 1
            noise_std = 0.05 if self.simulation else 0.0
            wind = WindEnvironment.get_wind(now) if self.simulation else (0.0, 0.0, 0.0)
            my_pos = tuple(self.ins.position)
            my_vel = tuple(self.ins.velocity)
            m_target = tuple(self.mission_target) if hasattr(self, 'mission_target') else None
            battery_low = self.battery_pct < 0.1
            target_v = self.swarm_logic.calculate_control_output(my_pos=my_pos, my_vel=my_vel, mission_target=m_target, dt=dt, current_time=now, battery_low=battery_low)
            self.ins.simulate_update(dt, list(target_v), noise_std=noise_std, wind=wind)
            self.battery_pct = max(0, self.battery_pct - 0.0001)
            if packet_drop_prob > 0 and random.random() < packet_drop_prob:
                continue
            if delay_ms > 0:
                await asyncio.sleep(delay_ms / 1000.0)
            if self.gcs_conn:
                if now - last_hb > 1.0:
                    self.gcs_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED | (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0), 0, mavutil.mavlink.MAV_STATE_ACTIVE if self.armed else mavutil.mavlink.MAV_STATE_STANDBY)
                    last_hb = now
                if tick % 4 == 0:
                    pos = self.ins.position
                    vel = self.ins.velocity
                    att = self.ins.attitude
                    self.gcs_conn.mav.local_position_ned_send(int(now * 1000) & 4294967295, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2])
                    lat, lon = (12.9716, 77.5946)
                    plat = int((lat + pos[0] * 9e-06) * 10000000.0)
                    plon = int((lon + pos[1] * 9e-06) * 10000000.0)
                    self.gcs_conn.mav.global_position_int_send(int(now * 1000) & 4294967295, plat, plon, int(-pos[2] * 1000), int(-pos[2] * 1000), int(vel[0] * 100), int(vel[1] * 100), int(vel[2] * 100), int(att[2] * 100))
                if tick % 2 == 0:
                    att = self.ins.attitude
                    self.gcs_conn.mav.attitude_send(int(now * 1000) & 4294967295, att[0], att[1], att[2], 0, 0, 0)
                if tick % 10 == 0:
                    self.gcs_conn.mav.sys_status_send(0, 0, 0, 500, 11100, int(self.battery_pct * 10), 0, 0, 0, 0, 0, 0, 0)

    def status(self) -> dict:
        return {'drone_id': self.drone_id, 'system_id': self.system_id, 'armed': self.armed, 'mode': self.mode, 'battery_pct': round(self.battery_pct, 1), 'status_text': self.status_text, 'mission_active': self.mission_active, 'ins': self.ins.to_dict()}

class FlightDatabase:

    def __init__(self, db_path: str='swayam_flights.db'):
        if db_path == ':memory:':
            self.db_path = 'file:memdb1?mode=memory&cache=shared'
        else:
            self.db_path = db_path
        self._queue = asyncio.Queue(maxsize=5000)
        self.metrics = {'dropped_telemetry': 0, 'last_drop_time': 0}
        self._init_schema()
        self._writer_task = None

    def start_writer(self):
        if not self._writer_task:
            self._writer_task = asyncio.create_task(self._db_writer())

    async def stop_writer(self):
        if self._writer_task:
            self._writer_task.cancel()
            try:
                await self._writer_task
            except asyncio.CancelledError:
                pass

    async def _db_writer(self):
        uri = self.db_path.startswith('file:')
        conn = sqlite3.connect(self.db_path, isolation_level=None, uri=uri)
        conn.execute('PRAGMA journal_mode=WAL;')
        batch = []
        while True:
            try:
                item = await self._queue.get()
                batch.append(item)
                if len(batch) >= 100 or self._queue.empty():
                    for qry, args in batch:
                        conn.execute(qry, args)
                    batch.clear()
            except asyncio.CancelledError:
                for qry, args in batch:
                    conn.execute(qry, args)
                break
            except Exception as e:
                logger.error(f'DB Write error: {e}')

    def _conn(self) -> sqlite3.Connection:
        uri = self.db_path.startswith('file:')
        conn = sqlite3.connect(self.db_path, check_same_thread=False, uri=uri)
        conn.row_factory = sqlite3.Row
        # TODO: should probably use a connection pool here. this works for now.
        return conn

    def _init_schema(self):
        c = self._conn()
        c.execute('PRAGMA journal_mode=WAL;')
        c.executescript("\n        CREATE TABLE IF NOT EXISTS flight_logs (\n            id          INTEGER PRIMARY KEY AUTOINCREMENT,\n            drone_id    TEXT NOT NULL,\n            timestamp   REAL NOT NULL,\n            level       TEXT DEFAULT 'INFO',\n            event       TEXT NOT NULL,\n            details     TEXT\n        );\n        CREATE TABLE IF NOT EXISTS ins_telemetry (\n            id          INTEGER PRIMARY KEY AUTOINCREMENT,\n            drone_id    TEXT NOT NULL,\n            timestamp   REAL NOT NULL,\n            pos_n       REAL, pos_e REAL, pos_d REAL,\n            vel_n       REAL, vel_e REAL, vel_d REAL,\n            roll        REAL, pitch REAL, yaw REAL\n        );\n        CREATE TABLE IF NOT EXISTS missions (\n            id          INTEGER PRIMARY KEY AUTOINCREMENT,\n            drone_id    TEXT NOT NULL,\n            start_time  REAL NOT NULL,\n            end_time    REAL,\n            status      TEXT DEFAULT 'PLANNED',\n            path_json   TEXT,\n            notes       TEXT\n        );\n        CREATE INDEX IF NOT EXISTS idx_logs_drone ON flight_logs(drone_id);\n        CREATE INDEX IF NOT EXISTS idx_telem_drone ON ins_telemetry(drone_id);\n        ")
        c.commit()

    def _enqueue(self, qry, args):
        try:
            self._queue.put_nowait((qry, args))
        except asyncio.QueueFull:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait((qry, args))
                self.metrics['dropped_telemetry'] = self.metrics.get('dropped_telemetry', 0) + 1
                self.metrics['total_drops'] = self.metrics.get('total_drops', 0) + 1
                now = time.time()
                if now - self.metrics.get('last_drop_time', now) >= 1.0:
                    drops_per_sec = self.metrics['dropped_telemetry'] / max(1e-05, now - self.metrics['last_drop_time'])
                    logger.warning(f"DB Queue full. Drops/sec: {drops_per_sec:.1f}. Total drops: {self.metrics['total_drops']}")
                    self.metrics['dropped_telemetry'] = 0
                    self.metrics['last_drop_time'] = now
            except Exception:
                pass

    def log_event(self, drone_id: str, event: str, details: str='', level: str='INFO'):
        self._enqueue('INSERT INTO flight_logs (drone_id, timestamp, level, event, details) VALUES (?,?,?,?,?)', (drone_id, time.time(), level, event, details))

    def log_ins(self, drone_id: str, ins: ESKFStateInput):
        self._enqueue('INSERT INTO ins_telemetry\n               (drone_id, timestamp, pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, roll, pitch, yaw)\n               VALUES (?,?,?,?,?,?,?,?,?,?,?)', (drone_id, ins.timestamp, *ins.position, *ins.velocity, *ins.attitude))

    def create_mission(self, drone_id: str, path: List[Tuple[int, int]], notes: str='') -> int:
        c = self._conn()
        cur = c.execute('INSERT INTO missions (drone_id, start_time, status, path_json, notes) VALUES (?,?,?,?,?)', (drone_id, time.time(), 'ACTIVE', json.dumps(path), notes))
        c.commit()
        return cur.lastrowid

    def complete_mission(self, mission_id: int, status: str='COMPLETED'):
        c = self._conn()
        c.execute('UPDATE missions SET end_time=?, status=? WHERE id=?', (time.time(), status, mission_id))
        c.commit()

    def get_recent_logs(self, drone_id: Optional[str]=None, limit: int=100) -> List[dict]:
        c = self._conn()
        if drone_id:
            rows = c.execute('SELECT * FROM flight_logs WHERE drone_id=? ORDER BY timestamp DESC LIMIT ?', (drone_id, limit)).fetchall()
        else:
            rows = c.execute('SELECT * FROM flight_logs ORDER BY timestamp DESC LIMIT ?', (limit,)).fetchall()
        return [dict(r) for r in rows]

    def get_ins_history(self, drone_id: str, limit: int=200) -> List[dict]:
        c = self._conn()
        rows = c.execute('SELECT * FROM ins_telemetry WHERE drone_id=? ORDER BY timestamp DESC LIMIT ?', (drone_id, limit)).fetchall()
        return [dict(r) for r in rows]

    def export_json(self, path: str='swayam_export.json'):
        # dumps everything to json for post-flight analysis
        # warning: on long flights this can be like 200mb. you've been warned.
        data = {'exported_at': datetime.utcnow().isoformat(), 'flight_logs': self.get_recent_logs(limit=9999), 'ins_telemetry': []}
        c = self._conn()
        rows = c.execute('SELECT * FROM ins_telemetry ORDER BY timestamp').fetchall()
        data['ins_telemetry'] = [dict(r) for r in rows]
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        logger.info(f'Exported DB to {path}')

class SwayamFleet:

    def __init__(self, db_path: str='swayam_flights.db', map_size: int=50):
        self.drones: Dict[str, DroneAgent] = {}
        self.db = FlightDatabase(db_path)
        self.grid = GridMap(map_size, map_size)
        self.log = logging.getLogger('swayam.fleet')
        self.disturbance_start = None
        self.last_recovery_time = 0.0
        self.time_in_degraded_mode = 0.0
        self.last_metrics_time = time.time()

    def add_drone(self, drone_id: str, system_id: int=1, connection_string: str='udp:127.0.0.1:14550', gcs_url: Optional[str]=None, simulation: bool=True) -> DroneAgent:
        drone = DroneAgent(drone_id, system_id, connection_string, gcs_url, simulation)
        self.drones[drone_id] = drone
        self.db.log_event(drone_id, 'DRONE_REGISTERED', f'conn={connection_string} gcs={gcs_url} sim={simulation}')
        return drone

    def add_obstacle(self, x: int, y: int, radius: int=1):
        self.grid.add_obstacle(x, y, radius)
        self.log.info(f'Obstacle added at ({x},{y}) r={radius}')

    async def connect_all(self) -> Dict[str, bool]:
        results = {}
        for did, drone in self.drones.items():
            ok = await drone.connect()
            results[did] = ok
            self.db.log_event(did, 'CONNECTED' if ok else 'CONNECT_FAILED')
        return results

    async def disconnect_all(self):
        self.log.info('Disconnecting all drones...')
        for d in self.drones.values():
            await d.disconnect()

    def compute_global_metrics(self) -> dict:
        drones = [d for d in self.drones.values() if getattr(d, 'status', 'ACTIVE') != 'LOST']
        if len(drones) < 2:
            return {}
        min_sep = float('inf')
        formation_error = 0.0
        control_effort = 0.0
        vels_n = []
        vels_e = []
        total_violations = 0
        geofence_breaches = 0
        min_sep_violations = 0
        degraded = False
        for i, d1 in enumerate(drones):
            vel = d1.ins.velocity
            control_effort += math.hypot(vel[0], vel[1])
            vels_n.append(vel[0])
            vels_e.append(vel[1])
            total_violations += d1.swarm_logic.metrics.get('violation_count', 0)
            geofence_breaches += d1.swarm_logic.metrics.get('geofence_breaches', 0)
            min_sep_violations += d1.swarm_logic.metrics.get('min_sep_violations', 0)
            if d1.swarm_logic.rtl_triggered or len(d1.swarm_logic.swarm_state) == 0:
                degraded = True
            for j, d2 in enumerate(drones):
                if i >= j:
                    continue
                dist = math.hypot(d1.ins.position[0] - d2.ins.position[0], d1.ins.position[1] - d2.ins.position[1])
                if dist > 0.0:
                    min_sep = min(min_sep, dist)
                formation_error += abs(dist - 5.0)
        vel_var = 0.0
        if len(vels_n) > 1:
            mean_n = sum(vels_n) / len(vels_n)
            mean_e = sum(vels_e) / len(vels_e)
            var_n = sum(((v - mean_n) ** 2 for v in vels_n)) / len(vels_n)
            var_e = sum(((v - mean_e) ** 2 for v in vels_e)) / len(vels_e)
            vel_var = var_n + var_e
        form_err_avg = formation_error / max(1, len(drones))
        now = time.time()
        dt = now - self.last_metrics_time
        self.last_metrics_time = now
        if degraded:
            self.time_in_degraded_mode += dt
        if form_err_avg > 10.0:
            if self.disturbance_start is None:
                self.disturbance_start = now
        elif self.disturbance_start is not None:
            self.last_recovery_time = now - self.disturbance_start
            self.disturbance_start = None
        return {'min_separation': round(min_sep if min_sep != float('inf') else 0.0, 2), 'formation_error': round(form_err_avg, 2), 'control_effort': round(control_effort, 2), 'vector_variance': round(vel_var, 4), 'recovery_time': round(self.last_recovery_time, 2), 'geofence_breaches': geofence_breaches, 'min_sep_violations': min_sep_violations, 'degraded_time': round(self.time_in_degraded_mode, 2)}

    def reassign_mission(self, failed_drone_id: str):
        failed_drone = self.drones.get(failed_drone_id)
        if not failed_drone or not failed_drone.mission_active:
            return
        max_capacity = 3
        available_drones = []
        for did, d in self.drones.items():
            if did != failed_drone_id and getattr(d, 'status', 'ACTIVE') != 'LOST':
                current_workload = 1 if d.mission_active else 0
                if current_workload < max_capacity:
                    dist = math.hypot(d.ins.position[0] - failed_drone.ins.position[0], d.ins.position[1] - failed_drone.ins.position[1])
                    score = dist + current_workload * 10.0
                    available_drones.append((score, d))
        available_drones.sort(key=lambda x: x[0])
        if available_drones:
            best_drone = available_drones[0][1]
            self.log.info(f'Reassigning mission from {failed_drone_id} to {best_drone.drone_id} in batch.')
            best_drone.mission_active = True
            failed_drone.mission_active = False
        if best_drone:
            self.log.info(f'Reassigning mission from {failed_drone_id} to {best_drone.drone_id} (score={min_score:.1f})')

    def plan_path(self, drone_id: str, goal_n: float, goal_e: float) -> List[Tuple[int, int]]:
        drone = self.drones[drone_id]
        sx = int(drone.ins.position[0]) + self.grid.width // 2
        sy = int(drone.ins.position[1]) + self.grid.height // 2
        gx = int(goal_n) + self.grid.width // 2
        gy = int(goal_e) + self.grid.height // 2
        sx, sy = (max(0, min(sx, self.grid.width - 1)), max(0, min(sy, self.grid.height - 1)))
        gx, gy = (max(0, min(gx, self.grid.width - 1)), max(0, min(gy, self.grid.height - 1)))
        path = self.grid.astar((sx, sy), (gx, gy))
        self.log.info(f'[{drone_id}] Path planned: {len(path)} waypoints')
        return path

    async def execute_mission(self, drone_id: str, goal_n: float, goal_e: float, altitude: float=10.0, speed: float=2.0):
        drone = self.drones[drone_id]
        path = self.plan_path(drone_id, goal_n, goal_e)
        if not path:
            self.db.log_event(drone_id, 'MISSION_ABORT', 'No path found', 'ERROR')
            return
        mission_id = self.db.create_mission(drone_id, path, f'goal=({goal_n},{goal_e})')
        self.db.log_event(drone_id, 'MISSION_START', f'goal=({goal_n},{goal_e}) wpts={len(path)}')
        try:
            await drone.takeoff(altitude)
            self.db.log_event(drone_id, 'TAKEOFF', f'alt={altitude}m')
            drone.mission_active = True
            for i, (wx, wy) in enumerate(path):
                confidence = drone.ins.confidence_weight
                if confidence == 0.0:
                    self.db.log_event(drone_id, 'SENSOR_FAULT', 'ESKF FAULT. Freezing position.', 'ERROR')
                    drone.set_mode('LAND')
                    break
                elif confidence < 1.0:
                    self.db.log_event(drone_id, 'SENSOR_WARNING', 'ESKF WARNING. Covariance inflated.', 'WARNING')
                north = wx - self.grid.width // 2
                east = wy - self.grid.height // 2
                drone.send_ned_setpoint(north, east, -altitude)
                drone.current_waypoint = i
                self.db.log_ins(drone_id, drone.ins)
                self.db.log_event(drone_id, 'WAYPOINT', f'wp={i} N={north} E={east}')
                my_pos = drone.ins.position
                for other_id, other_drone in self.drones.items():
                    if other_id != drone_id:
                        other_pos = other_drone.ins.position
                        rel_n = other_pos[0] - my_pos[0]
                        rel_e = other_pos[1] - my_pos[1]
                        rel_d = other_pos[2] - my_pos[2]
                        self.db.log_event(drone_id, 'REL_POS', f'to={other_id} rel=({rel_n:.2f}, {rel_e:.2f}, {rel_d:.2f})', level='DEBUG')
                await asyncio.sleep(1.0 / speed)
            await drone.land()
            self.db.log_event(drone_id, 'LANDED')
            drone.mission_active = False
            self.db.complete_mission(mission_id, 'COMPLETED')
        except Exception as e:
            self.db.log_event(drone_id, 'MISSION_ERROR', str(e), 'ERROR')
            self.db.complete_mission(mission_id, 'FAILED')
            raise

    async def broadcast_destination(self, goal_n: float, goal_e: float, altitude: float=10.0):
        tasks = []
        for did in self.drones:
            t = asyncio.create_task(self.execute_mission(did, goal_n, goal_e, altitude))
            tasks.append(t)
        await asyncio.gather(*tasks)
        self.log.info('Broadcast mission complete.')

    def fleet_status(self) -> List[dict]:
        return [d.status() for d in self.drones.values()]

    def emergency_land_all(self):
        self.log.warning('EMERGENCY LAND — ALL DRONES')
        for did, drone in self.drones.items():
            drone.set_mode('LAND')
            self.db.log_event(did, 'EMERGENCY_LAND', level='ERROR')
if __name__ == '__main__':
    # quick sanity check, not a real test
    # use tests/ for actual testing
    async def main():
        fleet = SwayamFleet(db_path='swayam_flights.db')
        fleet.db.start_writer()
        fleet.add_obstacle(10, 10, radius=2)
        fleet.add_obstacle(20, 15, radius=3)
        fleet.add_obstacle(30, 25, radius=2)
        fleet.add_drone('ALPHA', system_id=1, simulation=True)
        fleet.add_drone('BETA', system_id=2, simulation=True)
        fleet.add_drone('GAMMA', system_id=3, simulation=True)
        await fleet.connect_all()
        missions = [('ALPHA', 15.0, 12.0, 10.0), ('BETA', -8.0, 18.0, 15.0), ('GAMMA', 5.0, -5.0, 12.0)]
        tasks = []
        for drone_id, gn, ge, alt in missions:
            t = asyncio.create_task(fleet.execute_mission(drone_id, gn, ge, alt))
            tasks.append(t)
        await asyncio.gather(*tasks)
        import os
        os.makedirs('logs', exist_ok=True)
        fleet.db.export_json('logs/swayam_export.json')
        print('\n=== Fleet Status ===')
        for s in fleet.fleet_status():
            print(json.dumps(s, indent=2))
        await fleet.disconnect_all()
        await fleet.db.stop_writer()
        print('done')
    asyncio.run(main())
