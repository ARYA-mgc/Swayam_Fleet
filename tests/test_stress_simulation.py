import pytest
import asyncio
import sys, os, random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from swayam_core import SwayamFleet

@pytest.mark.asyncio
async def test_stress_simulation():
    fleet = SwayamFleet(":memory:")
    fleet.db.start_writer()
    
    # 10 Drones
    for i in range(10):
        fleet.add_drone(f"DRONE_{i}", system_id=i, simulation=True)
    
    results = await fleet.connect_all()
    assert all(results.values())
    
    # Broadcast to short path to prevent super long test run
    await fleet.broadcast_destination(2.0, 2.0, altitude=5.0)
    
    await fleet.disconnect_all()
    await fleet.db.stop_writer()
    # If we get here without deadlocks, the async queue handles the load
    assert True
