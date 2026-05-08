#!/usr/bin/env python3
# hammers the swarm with 10 drones + cpu load + packet drops
# runs on my machine, no guarantee it works on yours
# warning: will spin up a cpu hog process. your laptop will sound like a plane.

import sys
import os
import time
import math
import asyncio
import threading
import multiprocessing
import argparse

sys.path.insert(0, os.path.dirname(__file__))
from core import SwayamFleet, DroneAgent

def cpu_hog():
    # chews cpu so we can see if the async loop starves
    import random
    while True:
        _ = sum([math.sqrt(i) for i in range(5000)])
        time.sleep(random.uniform(0.001, 0.01))

async def stress(duration_minutes: float):
    print("[stress] starting, this will take a while")
    print(f"[stress] running for {duration_minutes} minutes")
    print("[stress] ctrl-c to abort early")
    print("")

    fleet = SwayamFleet(db_path=":memory:")
    
    # Spawn 10 Drones in a grid to avoid initial collision
    print("[INIT] Spawning 10 drones with network delays and noise...")
    for i in range(1, 11):
        drone_id = f"DRONE_{i:02d}"
        drone = fleet.add_drone(drone_id, system_id=i, simulation=True)
        # Give drones a safe initial 5m separation grid
        row = (i - 1) // 3
        col = (i - 1) % 3
        drone.ins._last_valid_pos = [row * 5.0, col * 5.0, 0.0]
        # We manually hijack the _sim_telemetry to inject noise in this stress test
        # We replace the task running _sim_telemetry with our parameters.

    await fleet.connect_all()
    
    # Re-launch sim_telemetry with noise parameters
    for drone in fleet.drones.values():
        for task in drone._tasks:
            task.cancel()
        drone._tasks = [asyncio.create_task(drone._sim_telemetry(packet_drop_prob=0.1, delay_ms=50.0))]

    print("[STRESS] Injecting CPU Saturation...")
    p = multiprocessing.Process(target=cpu_hog)
    p.daemon = True
    p.start()

    duration_secs = duration_minutes * 60.0
    start_time = time.time()
    
    print(f"\n[RUN] Starting {duration_minutes}-minute continuous stability test...")
    try:
        while time.time() - start_time < duration_secs:
            elapsed = time.time() - start_time
            metrics = fleet.compute_global_metrics()
            
            queue_size = fleet.db._queue.qsize() if hasattr(fleet.db, '_queue') else 0
            total_drops = fleet.db.metrics.get("total_drops", 0)
            
            velocity_variance = 0.0
            time_to_recover = 0.0
            # For brevity in the script, we log basic metrics, but let's grab the extended ones
            vel_variance = metrics.get('velocity_variance', 0.0)
            rec_time = metrics.get('recovery_time', 0.0)
            
            print(f"[{elapsed:05.1f}s] Global Metrics: "
                  f"MinSep: {metrics.get('min_separation', 0):.2f}m | "
                  f"FormErr: {metrics.get('formation_error', 0):.2f} | "
                  f"VecVar: {metrics.get('vector_variance', 0.0):.2f} | "
                  f"RecTime: {metrics.get('recovery_time', 0.0):.1f}s | "
                  f"DegTime: {metrics.get('degraded_time', 0.0):.1f}s | "
                  f"QDrops: {total_drops}")
                  
            await asyncio.sleep(5.0)
    except KeyboardInterrupt:
        print("\n[STOP] Test aborted by user.")
        
    print("\n[RESULT] Stopping simulation.")
    p.terminate()
    await fleet.disconnect_all()
    
    final_drops = fleet.db.metrics.get("total_drops", 0)
    print(f"total drops: {final_drops}")
    print("stress test done. check logs if anything looks off")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Swayam Stress Test")
    parser.add_argument("--duration", type=float, default=1.0, help="Duration in minutes (e.g. 10.0)")
    args = parser.parse_args()
    
    asyncio.run(stress(args.duration))
