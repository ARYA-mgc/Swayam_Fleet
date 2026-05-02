import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from swarm_autonomous_logic import AutonomousSwarmLogic

def test_collision_override_priority():
    logic = AutonomousSwarmLogic("D1")
    # D2 is extremely close (1.0m), inside critical radius 2.0
    logic.update_swarm_state("D2", {"pos_n": 1.0, "pos_e": 0.0, "pos_d": -10.0, "vel_n": -5.0, "vel_e": 0.0, "vel_d": 0.0})
    
    # Mission targets N=10, altitude -10 (NED)
    target = logic.calculate_control_output((0,0,-10.0), (0,0,0), (10,0,-10.0), 0.1)
    
    # Since D2 is at (1.0, 0), dp is (-1.0, 0). Emergency should trigger.
    # Emergency bypass: agg_n = -1.0 * 6.0/1.0 = -6.0, clamped to max_safe_velocity
    assert target[0] < 0 # Moving AWAY from D2, overriding mission
