# Swayam Fleet: Decentralized Swarm Coordination & Safety

[![tests](https://github.com/ARYA-mgc/Swayam_Fleet/actions/workflows/test.yml/badge.svg)](https://github.com/ARYA-mgc/Swayam_Fleet/actions/workflows/test.yml)
[![Ecosystem](https://img.shields.io/badge/Part%20of-INS--Drone--Pixhawk-blue)](https://github.com/ARYA-mgc/ins-drone-pixhawk)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Swayam Fleet is a decentralized coordination and safety stack designed for heterogeneous multi-UAV swarms. It abstracts the complexity of inter-drone communication and collision avoidance, allowing for high-level mission execution on Raspberry Pi 4 companion computers interfaced with ArduPilot-based flight controllers.

![Swarm Formation Flight](docs/assets/swarm_formation_flight.jpeg)
*Field deployment: Two-UAV formation flight utilizing decentralized velocity obstacles.*

---

## 🛠 Core Capabilities

### 1. Decentralized Coordination
Utilizes a hybrid **Reynolds Flocking + Velocity Obstacle (VO)** approach. Each agent independently computes its optimal velocity vector based on local telemetry from neighbors, ensuring no single point of failure.

### 2. Deterministic Safety Guards
Commanded velocities are passed through a **Control Barrier Function (CBF)** layer. This layer projects the potentially unsafe mission-level velocity into the nearest safe half-space, guaranteeing a minimum separation $d_{safe}$ even if the flocking logic fails or the operator sends a dangerous command.

### 3. Asynchronous Persistence
High-frequency INS and system telemetry are buffered and persisted to a local **SQLite (WAL mode)** database. This ensures a forensic-grade flight record for post-mission analysis without introducing latency into the real-time control loop.

### 4. Resilient Communication
UDP-based broadcaster with built-in packet deduplication and sequence tracking. The system is tuned for high-latency, lossy RF environments typical of long-range mesh networks.

---

## 🏗 System Architecture

```mermaid
graph TD
    classDef l3 fill:#1a1a2e,stroke:#f59e0b,stroke-width:2px,color:#fff;
    classDef l2 fill:#16213e,stroke:#10b981,stroke-width:2px,color:#fff;
    classDef l1 fill:#0f3460,stroke:#3b82f6,stroke-width:2px,color:#fff;
    classDef l0 fill:#111827,stroke:#ef4444,stroke-width:2px,color:#fff;
    
    subgraph Coordination_Stack [Coordination]
        Logic[Autonomous Logic<br>VO + Flocking]:::l2
        MP[Motion Planning<br>A* + GridMap]:::l2
    end

    subgraph Safety_Enforcement [Safety & Verification]
        CBF[CBF Projection<br>h(x) >= 0]:::l3
        Inv[Invariant Monitor<br>Geofence/Collision]:::l3
        Cert[Lyapunov Certificate<br>Stability Check]:::l3
    end

    Logic --> CBF
    MP --> CBF
    CBF --> Inv
    Inv --> Cert
    Cert --> Bridge[MAVLink Bridge]:::l1
    Bridge --> FCU((Pixhawk FCU)):::l0
```

---

## 📈 Performance & Verification

### Pre-Flight Diagnostics
The system enforces a strict pre-arm sequence, monitoring 48+ hardware and software status registers.

![Pre-Flight Status](docs/assets/preflight_status_check.png)
*Automated status validation: Red indicators signify safety interlocks preventing arming.*

### Mission Execution
Real-time tracking of multi-vehicle waypoints and formation convergence.

![Mission Planner GCS](docs/assets/mission_planner_gcs.jpg)
*Synchronized mission monitoring of 4 UAVs in a formation waypoint sequence.*

### Inter-Agent Stability
Post-flight analysis of inter-agent distances demonstrates the efficacy of the CBF and VO layers in maintaining safe separation under dynamic maneuvers.

![Distance Metrics](docs/assets/fleet_distance_metrics.jpg)
*Separation metrics: Maintaining >100m operational distance during high-speed transit.*

---

## 💻 Technical Implementation

### Mathematical Foundation: Safety Projection
The core safety guarantee relies on solving a Quadratic Programming (QP) problem at every control tick:
$$\min_{v_{safe}} \|v_{safe} - v_{cmd}\|^2$$
$$\text{subject to } \dot{h}(x, v_{safe}) + \alpha h(x) \geq 0$$
where $h(x)$ defines the safe set (e.g., minimum separation).

### Directory Layout
```text
src/
  swayam/
    core/         # Coordinator, DroneAgent, SQLite WAL Persistence
    control/      # VO Logic, CBF Projection, Lyapunov Verification
    comms/        # MAVLink Bridges, UDP Telemetry, AES Encryption
    hardware/     # RPi4 entry points, Health Monitoring, GPIO
  scripts/        # SITL Simulation & Stress-Testing scripts
tests/            # Exhaustive Pytest suite (Mocking & Integration)
```

---

## 🚀 Quick Start

1. **Clone & Install**:
   ```bash
   git clone https://github.com/ARYA-mgc/Swayam_Fleet.git
   pip install -r requirements.txt
   ```
2. **Launch SITL**:
   ```bash
   python src/scripts/sim.py
   ```
3. **Connect GCS**: Connect Mission Planner to `127.0.0.1:14550` (UDP).

---

## 🗺 Roadmap
- [ ] **3D Velocity Obstacles**: Extension of avoidance cones into vertical dimensions.
- [ ] **MAVLink-FTP Integration**: High-speed binary log retrieval for post-flight forensics.
- [x] **NavCore Fusion**: EKF-based state estimation shared across the swarm.
- [x] **Hardened Safety**: CBF-based velocity projection and Lyapunov stability verification.

---

## 🔬 Experimental Results

Final analysis of swarm convergence under lossy network conditions (94% PDR).

![Swarm Trajectories](docs/assets/drones-05-00033-g009-550.jpg)
*High-fidelity 3D trajectory reconstruction showing global waypoint convergence for a 7-agent swarm.*

---
MIT License | Built with 🛠 by [ARYA-mgc](https://github.com/ARYA-mgc)
