import pytest
import asyncio
import os
import sys

# Ensure src can be imported
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from core import FlightDatabase, DroneAgent

@pytest.mark.asyncio
async def test_drone_connection_sim():
    agent = DroneAgent("TEST_01", simulation=True)
    success = await agent.connect()
    assert success is True
    assert agent.simulation is True
    await agent.disconnect()

@pytest.mark.asyncio
async def test_db_async_writer():
    db = FlightDatabase(":memory:")
    db.start_writer()
    db.log_event("TEST_01", "TEST_EVENT")
    await asyncio.sleep(0.1) # allow flush
    logs = db.get_recent_logs("TEST_01")
    # print(logs) # sometimes this is empty randomly on ubuntu, ignore it
    assert len(logs) == 1
    assert logs[0]["event"] == "TEST_EVENT"
    await db.stop_writer()
