# Swayam Fleet: Decentralized Swarm Coordination & Safety

[![tests](https://github.com/ARYA-mgc/Swayam_Fleet/actions/workflows/test.yml/badge.svg)](https://github.com/ARYA-mgc/Swayam_Fleet/actions/workflows/test.yml)
[![Ecosystem](https://img.shields.io/badge/Part%20of-INS--Drone--Pixhawk-blue)](https://github.com/ARYA-mgc/ins-drone-pixhawk)

Swayam Fleet is a production-grade decentralized coordination stack for Raspberry Pi 4 companion computers interfaced with Pixhawk/ArduPilot flight controllers. It provides robust collision avoidance, flocking logic, and deterministic safety guarantees for multi-UAV swarms operating in GPS-denied or degraded environments.

![Swarm Formation Flight](docs/assets/swarm_formation_flight.jpeg)
*Actual field deployment demonstrating synchronized formation flight.*

---

## Core Capabilities

- **Decentralized Flocking**: Implements Reynolds flocking (Cohesion, Separation, Alignment) combined with Velocity Obstacle (VO) based collision avoidance.
- **Safety Projection**: Control Barrier Functions (CBF) project commanded velocities into a safe half-space to enforce a 1.5m minimum separation distance at the firmware level.
- **Robust Telemetry**: UDP-based inter-drone state sharing with built-in tolerance for packet loss (validated up to 94% reception rate).
- **Hardened Safety**: Real-time invariant monitoring and Lyapunov-based stability verification.
- **GCS Integration**: Transparent MAVLink multiplexing for Mission Planner/QGroundControl.

---

## Performance Proof

### Swarm Convergence
The system demonstrates high-fidelity tracking of swarm centers and individual agent separation even under network jitter.

![Swarm Trajectories](docs/assets/drones-05-00033-g009-550.jpg)
*3D trajectory analysis showing convergence to waypoints with 94% packet reception.*

![Distance Metrics](docs/assets/fleet_distance_metrics.jpg)
*Inter-agent distance tracking for a 7-UAV swarm.*

### GCS Observation
Full compatibility with standard ground control stations for real-time monitoring of swarm state.

![Mission Planner GCS](docs/assets/mission_planner_gcs.jpg)
*Real-time monitoring in Mission Planner showing 4-UAV synchronized mission execution.*

---

## Project Structure

```text
src/
  swayam/
    core/
      core.py              # Fleet coordinator, Drone agent, SQLite persistence
      navcore/             # ESKF state publisher (Shared with ins-drone-pixhawk)
    control/
      logic.py             # VO avoidance, flocking, PID, geofence
      safety.py            # CBF, Lyapunov certificates, invariant guards
      mission.py           # Waypoint mission management
      mp.py                # Motion planning utilities
    comms/
      mav.py               # Robust Pi-to-Cube MAVLink bridge
      bridge.py            # INS-to-MAVLink message mapping
      relay.py             # GCS MAVLink multiplexer
      telem.py             # UDP broadcast layer
      sync.py              # Clock sync & state consistency
      sec.py               # AES-GCM encrypted comms
      cmds.py              # Internal command definitions
    hardware/
      node.py              # Main RPi4 entry point
      hw.py                # Hardware abstraction & GPIO
      health.py            # System health & watchdog
  scripts/
    sim.py                 # SITL simulation environment
    stress.py              # Performance & latency stress testing
tests/                     # Comprehensive pytest suite (57+ tests)
docs/assets/               # Technical assets, plots, and flight logs
```

---

## Implementation Details

### Safety Architecture

```mermaid
graph TD
    classDef l3 fill:#1a1a2e,stroke:#f59e0b,stroke-width:2px,color:#fff;
    classDef l2 fill:#16213e,stroke:#10b981,stroke-width:2px,color:#fff;
    classDef l1 fill:#0f3460,stroke:#3b82f6,stroke-width:2px,color:#fff;
    classDef l0 fill:#111827,stroke:#ef4444,stroke-width:2px,color:#fff;
    
    subgraph Swarm_Logic [Coordination]
        Logic[Autonomous Logic<br>VO + Flocking]:::l2
        Nav[Path Planning<br>A* + GridMap]:::l2
    end

    subgraph Safety_Projection [Safety Layer]
        CBF[CBF Projection<br>h(x) >= 0]:::l3
        Invariant[Invariant Monitor<br>Geofence/Collision]:::l3
    end

    Logic --> CBF
    Nav --> CBF
    CBF --> Invariant
    Invariant --> FCU((Pixhawk FCU)):::l0
```

The **Control Barrier Function (CBF)** implementation ensures that for any two drones $i$ and $j$:
$$h(x) = \|p_i - p_j\|^2 - d_{safe}^2 \geq 0$$
Commands are projected iteratively to satisfy this condition before being sent to the flight controller.

---

## Deployment & Verification

### Simulation (SITL)
Run the high-fidelity swarm simulation:
```bash
python src/scripts/sim.py
```

### Pre-Flight Checks
The system performs exhaustive Pre-Arm checks to ensure all subsystems (INS, Comms, Safety) are healthy.

![Pre-Flight Status](docs/assets/preflight_status_check.png)
*Exhaustive pre-flight status validation in Mission Planner.*

### Hardware Connection
```python
from src.swayam.core.core import SwayamFleet

fleet = SwayamFleet()
fleet.add_drone("ALPHA", system_id=1, connection_string="/dev/ttyAMA0,921600")
await fleet.connect_all()
```

---

## Roadmap

- [ ] 3D Velocity Obstacles (Altitude-aware avoidance)
- [ ] MAVLink-FTP integration for log retrieval
- [x] Decentralized ESKF (NavCore integration)
- [x] AES-GCM Encrypted Telemetry
- [x] CBF-based Safety Enforcement

---

## License
MIT — See [LICENSE](LICENSE)
