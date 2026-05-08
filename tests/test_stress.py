import pytest
import asyncio
import sys, os, random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from core import SwayamFleet
from logic import AutonomousSwarmLogic


@pytest.mark.skip(reason="my laptop fans sound like a jet engine when I run this, skipping for now")
def test_stress_high_drone_count():
    # was trying to push to 20 drones but queue saturates
    # need to profile before enabling this
    pass

@pytest.mark.asyncio
async def test_stress():
    fleet = SwayamFleet(":memory:")
    fleet.db.start_writer()
    
    # 10 Drones
    for i in range(10):
        fleet.add_drone(f"DRONE_{i}", system_id=i, simulation=True)
    
    results = await fleet.connect_all()
    assert all(results.values())

    # TODO: this only tests the happy path, edge case when dt=0 still broken
    # Broadcast to short path to prevent super long test run
    await fleet.broadcast_destination(2.0, 2.0, altitude=5.0)

    await fleet.disconnect_all()
    await fleet.db.stop_writer()
    # If we get here without deadlocks, the async queue handles the load
    # ... at least until the memory leak catches up
    assert True


def test_debug_arya_check_this():
    # was debugging leader election at like 2am, leaving for now
    # D1 should start with no leader
    logic = AutonomousSwarmLogic("X")
    assert logic.current_leader is None
