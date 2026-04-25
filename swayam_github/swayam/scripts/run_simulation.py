#!/usr/bin/env python3
"""
run_simulation.py — Run a complete Swayam simulation without hardware.
Spawns 3 drones, runs missions, saves logs to DB, exports JSON.

Usage:
    python scripts/run_simulation.py
"""

import sys
import os
import threading
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from swayam_core import SwayamFleet

def main():
    os.makedirs("logs", exist_ok=True)

    print("\n╔══════════════════════════════════════╗")
    print("║   SWAYAM  —  Simulation Mode         ║")
    print("╚══════════════════════════════════════╝\n")

    fleet = SwayamFleet(db_path="swayam_flights.db", map_size=50)

    # Obstacles
    fleet.add_obstacle(10, 10, radius=2)
    fleet.add_obstacle(20, 15, radius=3)
    fleet.add_obstacle(30, 25, radius=2)
    print("[MAP] 3 obstacles placed.")

    # Drones
    fleet.add_drone("ALPHA", system_id=1, simulation=True)
    fleet.add_drone("BETA",  system_id=2, simulation=True)
    fleet.add_drone("GAMMA", system_id=3, simulation=True)

    fleet.connect_all()
    print("[FLEET] 3 drones connected (simulation).\n")

    missions = [
        ("ALPHA", 15.0,  12.0, 10.0),
        ("BETA",  -8.0,  18.0, 15.0),
        ("GAMMA",  5.0,  -5.0, 12.0),
    ]

    threads = []
    for drone_id, gn, ge, alt in missions:
        print(f"[LAUNCH] {drone_id} → N={gn} E={ge} Alt={alt}m")
        t = threading.Thread(
            target=fleet.execute_mission,
            args=(drone_id, gn, ge, alt),
            daemon=False,
        )
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # Export
    export_path = "logs/swayam_export.json"
    fleet.db.export_json(export_path)
    print(f"\n[DB] Logs exported → {export_path}")

    print("\n=== Final Fleet Status ===")
    for s in fleet.fleet_status():
        ins = s["ins"]
        print(f"  {s['drone_id']} | Batt:{s['battery_pct']}% | "
              f"N={ins['position_N']:.2f} E={ins['position_E']:.2f} "
              f"Alt={-ins['position_D']:.2f}m")

    fleet.disconnect_all()
    print("\n✓ Simulation complete.\n")

if __name__ == "__main__":
    main()
