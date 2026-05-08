import pytest
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sync import SwarmCoordinator

@pytest.mark.asyncio
async def test_heartbeat_tracking():
    coord = SwarmCoordinator(["D1"])
    coord.update_heartbeat("D1", time.time(), seq=1)
    assert coord.drones["D1"]["last_seq"] == 1
    # print("hb works")

@pytest.mark.asyncio
async def test_packet_loss_calculation():
    coord = SwarmCoordinator(["D1"])
    coord.update_heartbeat("D1", time.time(), seq=1)
    coord.update_heartbeat("D1", time.time(), seq=5) # Missed 2,3,4
    # UDP is garbage on the pi zero, we drop packets all the time
    assert coord.drones["D1"]["total_dropped"] == 3
