#!/usr/bin/env python3
# quick sim runner, no hardware needed
# run this if you want to test the swarm without risking the actual drones

import sys
import os
import threading
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from core import SwayamFleet

def main():
    os.makedirs("logs", exist_ok=True)

    # used to have a fancy ascii banner here, removed it because it was embarrassing
    print("[sim] starting swarm simulation")
    print("[sim] connect mission planner to udp:14550 if you want to see something")

    fleet = SwayamFleet(db_path="swayam_flights.db", map_size=50)

    # Obstacles
    fleet.add_obstacle(10, 10, radius=2)
    fleet.add_obstacle(20, 15, radius=3)
    fleet.add_obstacle(30, 25, radius=2)
    print("[MAP] 3 obstacles placed.")

    # Drones (System IDs 1, 2, 3)
    # all broadcasting to same port because why not
    gcs = "udpout:127.0.0.1:14550"
    fleet.add_drone("ALPHA", system_id=1, gcs_url=gcs, simulation=True)
    fleet.add_drone("BETA",  system_id=2, gcs_url=gcs, simulation=True)
    fleet.add_drone("GAMMA", system_id=3, gcs_url=gcs, simulation=True)

    fleet.connect_all()
    # print("connected") # too much output

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
    print("done.")

if __name__ == "__main__":
    main()
