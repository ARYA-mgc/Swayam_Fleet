"""
tests/test_swayam.py — Unit tests for Swayam core components.
Run: pytest tests/
"""

import sys
import os
import math
import time
import tempfile
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from swayam_core import INSState, GridMap, DroneAgent, FlightDatabase, SwayamFleet


# ── INSState ────────────────────────────────────

class TestINSState:
    def test_initial_state(self):
        ins = INSState()
        assert ins.position == [0.0, 0.0, 0.0]
        assert ins.velocity == [0.0, 0.0, 0.0]
        assert ins.attitude == [0.0, 0.0, 0.0]

    def test_integrate_gravity_only(self):
        ins = INSState()
        # Pure gravity in body Z — should stay still in NED X/Y
        accel = [0.0, 0.0, -9.80665]
        gyro  = [0.0, 0.0, 0.0]
        ins.integrate(accel, gyro, dt=0.1)
        # X/Y should be near zero
        assert abs(ins.position[0]) < 0.01
        assert abs(ins.position[1]) < 0.01

    def test_integrate_forward(self):
        ins = INSState()
        # Accelerate forward (body X) with gravity zeroed
        accel = [1.0, 0.0, -9.80665]
        for _ in range(10):
            ins.integrate(accel, [0, 0, 0], dt=0.1)
        # Should move in +N direction
        assert ins.position[0] > 0

    def test_reset(self):
        ins = INSState()
        ins.position = [5.0, 3.0, -10.0]
        ins.velocity = [1.0, 2.0, 0.0]
        ins.reset()
        assert ins.position == [0.0, 0.0, 0.0]

    def test_to_dict_keys(self):
        ins = INSState()
        d = ins.to_dict()
        for key in ("position_N", "position_E", "position_D",
                    "velocity_N", "velocity_E", "velocity_D",
                    "roll_rad", "pitch_rad", "yaw_rad"):
            assert key in d

    def test_rotate_body_to_world_identity(self):
        ins = INSState()  # attitude = [0,0,0]
        result = ins.rotate_body_to_world([1.0, 0.0, 0.0])
        assert abs(result[0] - 1.0) < 1e-9

    def test_gyro_updates_attitude(self):
        ins = INSState()
        ins.integrate([0, 0, -9.80665], [0.1, 0, 0], dt=1.0)
        assert abs(ins.attitude[0] - 0.1) < 1e-6


# ── GridMap ─────────────────────────────────────

class TestGridMap:
    def test_empty_grid(self):
        g = GridMap(10, 10)
        assert g.is_free(0, 0)
        assert g.is_free(9, 9)

    def test_obstacle(self):
        g = GridMap(10, 10)
        g.add_obstacle(5, 5, radius=0)
        assert not g.is_free(5, 5)
        assert g.is_free(4, 5)

    def test_astar_straight(self):
        g = GridMap(10, 10)
        path = g.astar((0, 0), (5, 0))
        assert path[0] == (0, 0)
        assert path[-1] == (5, 0)
        assert len(path) >= 6

    def test_astar_with_obstacle(self):
        g = GridMap(10, 10)
        # Wall from y=0 to y=8 at x=5
        for y in range(9):
            g.add_obstacle(5, y)
        path = g.astar((0, 0), (7, 0))
        assert path  # should find path around
        assert path[-1] == (7, 0)

    def test_astar_blocked(self):
        g = GridMap(5, 5)
        # Surround goal
        for x in range(5):
            for y in range(5):
                if (x, y) != (0, 0):
                    g.add_obstacle(x, y)
        path = g.astar((0, 0), (4, 4))
        assert path == []

    def test_out_of_bounds(self):
        g = GridMap(10, 10)
        assert not g.is_free(-1, 0)
        assert not g.is_free(10, 10)

    def test_to_dict(self):
        g = GridMap(20, 20)
        d = g.to_dict()
        assert d["width"] == 20
        assert "obstacles" in d


# ── DroneAgent ──────────────────────────────────

class TestDroneAgent:
    def test_init(self):
        d = DroneAgent("TEST", simulation=True)
        assert d.drone_id == "TEST"
        assert not d.armed
        assert d.battery_pct == 100.0

    def test_connect_sim(self):
        d = DroneAgent("TEST", simulation=True)
        assert d.connect()
        d.disconnect()

    def test_arm_disarm(self):
        d = DroneAgent("TEST", simulation=True)
        d.connect()
        d.arm()
        assert d.armed
        d.disarm()
        assert not d.armed
        d.disconnect()

    def test_set_mode(self):
        d = DroneAgent("TEST", simulation=True)
        d.set_mode("GUIDED")
        assert d.mode == "GUIDED"

    def test_status_keys(self):
        d = DroneAgent("TEST", simulation=True)
        d.connect()
        s = d.status()
        for key in ("drone_id", "armed", "mode", "battery_pct", "ins"):
            assert key in s
        d.disconnect()

    def test_send_ned_setpoint_sim(self):
        d = DroneAgent("TEST", simulation=True)
        d.connect()
        d.send_ned_setpoint(10, 5, -10)
        # Position should update towards setpoint
        assert abs(d.ins.position[0]) > 0
        d.disconnect()


# ── FlightDatabase ───────────────────────────────

class TestFlightDatabase:
    def setup_method(self):
        self.tmp = tempfile.mktemp(suffix=".db")
        self.db = FlightDatabase(self.tmp)

    def teardown_method(self):
        if os.path.exists(self.tmp):
            os.remove(self.tmp)

    def test_log_event(self):
        self.db.log_event("DRONE1", "TEST_EVENT", "details here")
        logs = self.db.get_recent_logs("DRONE1")
        assert len(logs) == 1
        assert logs[0]["event"] == "TEST_EVENT"

    def test_log_ins(self):
        ins = INSState()
        ins.position = [1.0, 2.0, -10.0]
        self.db.log_ins("DRONE1", ins)
        history = self.db.get_ins_history("DRONE1")
        assert len(history) == 1
        assert abs(history[0]["pos_n"] - 1.0) < 1e-4

    def test_mission_lifecycle(self):
        mid = self.db.create_mission("DRONE1", [(0,0),(1,1)], "test")
        assert mid > 0
        self.db.complete_mission(mid, "COMPLETED")

    def test_export_json(self):
        self.db.log_event("DRONE1", "EV1")
        export_path = self.tmp + "_export.json"
        self.db.export_json(export_path)
        assert os.path.exists(export_path)
        with open(export_path) as f:
            data = json.load(f)
        assert "flight_logs" in data
        os.remove(export_path)

    def test_filter_by_drone(self):
        self.db.log_event("D1", "EV1")
        self.db.log_event("D2", "EV2")
        logs_d1 = self.db.get_recent_logs("D1")
        assert all(l["drone_id"] == "D1" for l in logs_d1)


# ── SwayamFleet ─────────────────────────────────

class TestSwayamFleet:
    def setup_method(self):
        self.tmp = tempfile.mktemp(suffix=".db")
        self.fleet = SwayamFleet(db_path=self.tmp)

    def teardown_method(self):
        self.fleet.disconnect_all()
        if os.path.exists(self.tmp):
            os.remove(self.tmp)

    def test_add_drone(self):
        d = self.fleet.add_drone("X", simulation=True)
        assert "X" in self.fleet.drones

    def test_connect_all(self):
        self.fleet.add_drone("A", simulation=True)
        self.fleet.add_drone("B", simulation=True)
        results = self.fleet.connect_all()
        assert results["A"] and results["B"]

    def test_fleet_status(self):
        self.fleet.add_drone("A", simulation=True)
        self.fleet.connect_all()
        status = self.fleet.fleet_status()
        assert len(status) == 1
        assert status[0]["drone_id"] == "A"

    def test_plan_path(self):
        self.fleet.add_drone("A", simulation=True)
        self.fleet.connect_all()
        path = self.fleet.plan_path("A", 5.0, 5.0)
        assert isinstance(path, list)

    def test_emergency_land(self):
        self.fleet.add_drone("A", simulation=True)
        self.fleet.connect_all()
        self.fleet.emergency_land_all()
        assert self.fleet.drones["A"].mode == "LAND"


import json  # needed for FlightDatabase test
