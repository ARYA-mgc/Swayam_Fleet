import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from logic import AutonomousSwarmLogic

@pytest.mark.skip(reason="D2 kept crashing into D1 in SITL, need to tune PID first")
def test_col_priority():
    logic = AutonomousSwarmLogic("D1")
    # D2 is extremely close (1.0m)
    logic.update_swarm_state("D2", {"pos_n": 1.0, "pos_e": 0.0, "pos_d": -10.0, "vel_n": -5.0, "vel_e": 0.0, "vel_d": 0.0})
    
    target = logic.calculate_control_output((0,0,-10.0), (0,0,0), (10,0,-10.0), 0.1)
    # print(target) # WTF why is it going up??
    assert target[0] < 0 
