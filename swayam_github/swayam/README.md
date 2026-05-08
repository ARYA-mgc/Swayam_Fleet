# Swayam — MAVLink Multi-Drone Fleet Management

> INS-guided, GPS-independent multi-drone coordination with A* path planning, SQLite telemetry logging, and a real-time web dashboard.

![CI](https://github.com/YOUR_USERNAME/swayam/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Features

| Feature | Details |
|---|---|
| **INS Navigation** | Dead-reckoning via IMU integration — body-to-world frame rotation, gravity compensation, velocity & position tracking. No GPS required. |
| **A\* Path Planning** | 8-directional A\* on a 50×50m occupancy grid with Euclidean heuristic and obstacle inflation. |
| **MAVLink Integration** | `SET_POSITION_TARGET_LOCAL_NED` setpoints, ARM/DISARM, mode switching, `SCALED_IMU2` telemetry. Works over UDP, TCP, or serial. |
| **Fleet Coordination** | N drones in parallel threads. Broadcast missions or individual assignments. Emergency land all. |
| **SQLite Database** | 3-table schema: `flight_logs`, `ins_telemetry`, `missions`. Thread-safe. JSON export. |
| **Web Dashboard** | Flask UI on `:5050` — live fleet cards, INS map, filterable log table, broadcast controls. Auto-refreshes every 4s. |
| **Simulation Mode** | Full simulation without hardware — synthetic IMU, path execution, DB writes. Great for CI and development. |

---

## Architecture

```
swayam/
├── src/
│   └── core.py          # Core library — 5 classes
├── dash.py         # Flask web dashboard
├── scripts/
│   └── sim.py       # Headless simulation runner
├── tests/
│   └── test_main.py          # 30+ pytest unit tests
├── logs/                       # Auto-created — DB exports
├── .github/
│   └── workflows/
│       └── ci.yml              # GitHub Actions CI
├── requirements.txt
└── README.md
```

### Core Classes (`src/core.py`)

```
INSState          Dead-reckoning navigation
GridMap + A*      Occupancy grid & optimal path planning
DroneAgent        Single drone — MAVLink, telemetry, missions
FlightDatabase    SQLite persistence layer
SwayamFleet       Fleet coordinator — parallel missions, logging
```

---

## Quick Start

### 1. Clone & Install

```bash
git clone https://github.com/YOUR_USERNAME/swayam.git
cd swayam
pip install -r requirements.txt
```

### 2. Run Simulation (no hardware needed)

```bash
python scripts/sim.py
```

Output:
```
╔══════════════════════════════════════╗
║   SWAYAM  —  Simulation Mode         ║
╚══════════════════════════════════════╝

[LAUNCH] ALPHA → N=15.0 E=12.0 Alt=10.0m
[LAUNCH] BETA  → N=-8.0 E=18.0 Alt=15.0m
[LAUNCH] GAMMA → N=5.0  E=-5.0 Alt=12.0m
...
✓ Simulation complete.
```

### 3. Launch Dashboard

```bash
python dash.py
# Open http://localhost:5050
```

### 4. Run Tests

```bash
pytest tests/ -v
```

---

## Connecting Real Hardware

### ArduPilot / Pixhawk (UDP)

```python
from src.core import SwayamFleet

fleet = SwayamFleet()
fleet.add_drone("ALPHA", system_id=1,
                connection_string="udp:192.168.1.10:14550",
                simulation=False)
fleet.connect_all()
fleet.execute_mission("ALPHA", goal_n=20.0, goal_e=15.0, altitude=10.0)
```

### STM32 + CAN-MAVLink Bridge (Serial)

```python
fleet.add_drone("BETA", system_id=2,
                connection_string="/dev/ttyUSB0,115200",
                simulation=False)
```

> **Note**: Uncomment `pymavlink>=2.4.37` in `requirements.txt` before connecting real hardware.

### SITL (Software in the Loop)

```bash
# Start ArduPilot SITL
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550

# Connect Swayam
python -c "
from src.core import SwayamFleet
f = SwayamFleet()
f.add_drone('SIM', connection_string='udp:127.0.0.1:14550', simulation=False)
f.connect_all()
f.execute_mission('SIM', 10, 10, 15)
"
```

---

## INS Navigation Detail

The `INSState` class implements a basic strapdown INS:

1. **Attitude Update** — Gyroscope integration (Euler angles):
   ```
   roll  += ωx · dt
   pitch += ωy · dt
   yaw   += ωz · dt
   ```

2. **Body→World Rotation** — Full ZYX Euler rotation matrix applied to accelerometer readings.

3. **Gravity Removal** — Subtract `g = 9.80665 m/s²` from world-frame Z (NED down).

4. **Velocity Integration**:
   ```
   v += a_world · dt
   ```

5. **Position Integration**:
   ```
   p += v · dt
   ```

INS data is fed from `SCALED_IMU2` MAVLink messages (units: milli-g, milli-rad/s).

---

## A* Path Planning

- Grid: 50×50 cells, 1m per cell
- Start: drone's current INS position (mapped to grid)
- Goal: target N/E coordinates
- Movement: 8-directional (allows diagonal)
- Heuristic: Euclidean distance (admissible → optimal)
- Obstacle support: radius-based inflation

```python
fleet.add_obstacle(x=10, y=15, radius=2)  # 5x5 blocked area
path = fleet.plan_path("ALPHA", goal_n=20, goal_e=18)
```

---

## Dashboard API

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Web dashboard |
| `/api/status` | GET | All drone status + INS |
| `/api/logs` | GET | Recent flight logs |
| `/api/ins/<drone_id>` | GET | INS telemetry history |
| `/api/mission` | POST | Broadcast mission `{goal_n, goal_e}` |
| `/api/emergency_land` | POST | Land all drones immediately |
| `/api/export` | GET | Export DB to JSON |

---

## Database Schema

```sql
flight_logs     (id, drone_id, timestamp, level, event, details)
ins_telemetry   (id, drone_id, timestamp, pos_n, pos_e, pos_d,
                                           vel_n, vel_e, vel_d,
                                           roll, pitch, yaw)
missions        (id, drone_id, start_time, end_time, status,
                               path_json, notes)
```

---

## CI/CD

GitHub Actions runs on every push:
1. Tests on Python 3.9, 3.10, 3.11
2. Coverage report
3. Full headless simulation
4. Uploads `swayam_export.json` as artifact

---

## Contributing

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Run tests: `pytest tests/ -v`
4. Submit a pull request

---

## License

MIT — see [LICENSE](LICENSE)

---

## Roadmap

- [ ] GPS/INS fusion (Extended Kalman Filter)
- [ ] 3D voxel grid for obstacle avoidance
- [ ] WebSocket live telemetry (replace polling)
- [ ] ROS 2 bridge node
- [ ] Geofence enforcement
- [ ] Multi-vehicle conflict resolution
