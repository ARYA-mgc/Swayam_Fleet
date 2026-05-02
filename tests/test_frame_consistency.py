import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from swarm_sync import SwarmCoordinator

def test_leader_failover_and_origin():
    coord = SwarmCoordinator(["D1", "D2"])
    coord.origin = (12.34, 56.78, 100)
    assert coord.leader_id == "D1" # D1 < D2
    
    # Simulate timeout for D1
    coord.drones["D1"]["status"] = "LOST"
    # advance time to bypass cooldown
    coord._last_election = 0 
    coord.elect_leader()
    
    assert coord.leader_id == "D2"
    # Origin remains intact
    assert coord.origin == (12.34, 56.78, 100)
