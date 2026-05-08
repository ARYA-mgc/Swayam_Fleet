import pytest
import math
import asyncio
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from core import SwayamFleet

@pytest.mark.asyncio
async def test_scenario_1_4_drone_cross():
    fleet = SwayamFleet(db_path=":memory:")
    fleet.add_drone("N", simulation=True)
    fleet.add_drone("S", simulation=True)
    fleet.add_drone("E", simulation=True)
    fleet.add_drone("W", simulation=True)
    
    from core import ESKFState
    
    await fleet.connect_all()
    
    def set_pos(d, px, py):
        st = ESKFState(t=time.time(), px=px, py=py, pz=-10.0, vx=0.0, vy=0.0, vz=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, cov_trace=0.1, health=1)
        d.ins._pub.publish(st)
        
    set_pos(fleet.drones["N"], 20.0, 0.0)
    set_pos(fleet.drones["S"], -20.0, 0.0)
    set_pos(fleet.drones["E"], 0.0, 20.0)
    set_pos(fleet.drones["W"], 0.0, -20.0)
    for d in fleet.drones.values():
        for t in d._tasks: t.cancel()
        d._tasks = []
        d.mission_target = (0, 0, 0) # Head to center
        
    dt = 0.1
    min_dist_observed = 999.0
    
    for step in range(300):
        now = step * dt
        # Manual sim step
        for d in fleet.drones.values():
            my_pos = tuple(d.ins.position)
            my_vel = tuple(d.ins.velocity)
            # Head to opposite side
            if d.drone_id == "N": m_target = (-20.0, 0.0, -10.0)
            elif d.drone_id == "S": m_target = (20.0, 0.0, -10.0)
            elif d.drone_id == "E": m_target = (0.0, -20.0, -10.0)
            elif d.drone_id == "W": m_target = (0.0, 20.0, -10.0)
            
            target_v = d.swarm_logic.calculate_control_output(
                my_pos=my_pos, my_vel=my_vel, mission_target=m_target, 
                dt=dt, current_time=now, max_safe_velocity=4.0, a_max=8.0
            )
            if step % 20 == 0:
                print(f"[{now:.1f}s] {d.drone_id} Pos: {my_pos[0]:.1f}, {my_pos[1]:.1f} | Vel: {my_vel[0]:.1f}, {my_vel[1]:.1f} | Cmd: {target_v[0]:.1f}, {target_v[1]:.1f}")
            d.ins.simulate_update(dt, list(target_v))
            
            # Broadcast state
            state = {"pos_n": d.ins.position[0], "pos_e": d.ins.position[1], "pos_d": d.ins.position[2],
                     "vel_n": d.ins.velocity[0], "vel_e": d.ins.velocity[1], "vel_d": d.ins.velocity[2],
                     "timestamp": now}
            for other in fleet.drones.values():
                if other != d:
                    other.swarm_logic.update_swarm_state(d.drone_id, state, dt, now)
                    
        # Check distances
        agents = list(fleet.drones.values())
        for i in range(len(agents)):
            for j in range(i+1, len(agents)):
                dist = math.hypot(agents[i].ins.position[0] - agents[j].ins.position[0],
                                  agents[i].ins.position[1] - agents[j].ins.position[1])
                min_dist_observed = min(min_dist_observed, dist)
                
    await fleet.disconnect_all()
    assert min_dist_observed >= 1.5, f"Collision detected! Min dist: {min_dist_observed}"

@pytest.mark.skip(reason="SITL crashes with 8 drones, need a better PC")
@pytest.mark.asyncio
async def test_scenario_2_dense_swarm():
    fleet = SwayamFleet(db_path=":memory:")
    from core import ESKFState
    await fleet.connect_all()
    
    def set_pos(d, px, py):
        st = ESKFState(t=time.time(), px=px, py=py, pz=-10.0, vx=0.0, vy=0.0, vz=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, cov_trace=0.1, health=1)
        d.ins._pub.publish(st)
        
    for i in range(8):
        d = fleet.add_drone(f"D{i}", simulation=True)
        set_pos(d, math.cos(i) * 15.0, math.sin(i) * 15.0)
    for d in fleet.drones.values():
        for t in d._tasks: t.cancel()
        d._tasks = []
    
    # Seed initial swarm state so collision avoidance is aware of neighbours from step 0
    for d in fleet.drones.values():
        state = {"pos_n": d.ins.position[0], "pos_e": d.ins.position[1], "pos_d": d.ins.position[2],
                 "vel_n": d.ins.velocity[0], "vel_e": d.ins.velocity[1], "vel_d": d.ins.velocity[2],
                 "timestamp": 0.0}
        for other in fleet.drones.values():
            if other != d:
                other.swarm_logic.update_swarm_state(d.drone_id, state, 0.0, 0.0)
        
    dt = 0.1
    min_dist_observed = 999.0
    
    for step in range(200):
        now = step * dt
        for d in fleet.drones.values():
            target_v = d.swarm_logic.calculate_control_output(
                my_pos=tuple(d.ins.position), my_vel=tuple(d.ins.velocity), 
                mission_target=(100.0, 100.0, -10.0), dt=dt, current_time=now,
                max_safe_velocity=4.0, a_max=8.0
            )
            d.ins.simulate_update(dt, list(target_v))
            
            state = {"pos_n": d.ins.position[0], "pos_e": d.ins.position[1], "pos_d": d.ins.position[2],
                     "vel_n": d.ins.velocity[0], "vel_e": d.ins.velocity[1], "vel_d": d.ins.velocity[2],
                     "timestamp": now}
            for other in fleet.drones.values():
                if other != d:
                    other.swarm_logic.update_swarm_state(d.drone_id, state, dt, now)
                    
        agents = list(fleet.drones.values())
        for i in range(len(agents)):
            for j in range(i+1, len(agents)):
                dist = math.hypot(agents[i].ins.position[0] - agents[j].ins.position[0],
                                  agents[i].ins.position[1] - agents[j].ins.position[1])
                min_dist_observed = min(min_dist_observed, dist)
                
    await fleet.disconnect_all()
    assert min_dist_observed >= 1.5

@pytest.mark.asyncio
async def test_scenario_3_delayed_telemetry():
    fleet = SwayamFleet(db_path=":memory:")
    fleet.add_drone("N", simulation=True)
    fleet.add_drone("S", simulation=True)
    
    from core import ESKFState
    
    await fleet.connect_all()
    
    def set_pos(d, px, py):
        st = ESKFState(t=time.time(), px=px, py=py, pz=-10.0, vx=0.0, vy=0.0, vz=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, cov_trace=0.1, health=1)
        d.ins._pub.publish(st)
        
    set_pos(fleet.drones["N"], 15.0, 0.0)
    set_pos(fleet.drones["S"], -15.0, 0.0)
    for d in fleet.drones.values():
        for t in d._tasks: t.cancel()
        d._tasks = []
        
    dt = 0.1
    min_dist_observed = 999.0
    
    for step in range(200):
        now = step * dt
        for d in fleet.drones.values():
            m_target = (-15.0, 0.0, -10.0) if d.drone_id == "N" else (15.0, 0.0, -10.0)
            target_v = d.swarm_logic.calculate_control_output(
                my_pos=tuple(d.ins.position), my_vel=tuple(d.ins.velocity), 
                mission_target=m_target, dt=dt, current_time=now,
                max_safe_velocity=4.0, a_max=8.0
            )
            if step % 20 == 0 and d.drone_id == "N":
                print(f"[Scen3 {now:.1f}s] {d.drone_id} Pos: {d.ins.position[0]:.1f}, {d.ins.position[1]:.1f} | Vel: {d.ins.velocity[0]:.1f}, {d.ins.velocity[1]:.1f} | Cmd: {target_v[0]:.1f}, {target_v[1]:.1f}")
            d.ins.simulate_update(dt, list(target_v))
            
            # Delayed broadcast (150ms = roughly every 2 steps at dt=0.1)
            if step % 2 == 0:
                state = {"pos_n": d.ins.position[0], "pos_e": d.ins.position[1], "pos_d": d.ins.position[2],
                         "vel_n": d.ins.velocity[0], "vel_e": d.ins.velocity[1], "vel_d": d.ins.velocity[2],
                         "timestamp": now}
                for other in fleet.drones.values():
                    if other != d:
                        other.swarm_logic.update_swarm_state(d.drone_id, state, dt, now)
                        
        dist = math.hypot(fleet.drones["N"].ins.position[0] - fleet.drones["S"].ins.position[0],
                          fleet.drones["N"].ins.position[1] - fleet.drones["S"].ins.position[1])
        min_dist_observed = min(min_dist_observed, dist)
        
    await fleet.disconnect_all()
    assert min_dist_observed >= 1.5
