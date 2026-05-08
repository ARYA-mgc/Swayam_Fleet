# just a dumb dataclass to hold the eskf state so we dont pass 15 floats around
# who thought 15 floats was a good idea anyway
# if anyone adds another float to this I'm quitting
from dataclasses import dataclass
import time

@dataclass
class ESKFState:
    t: float
    px: float
    py: float
    pz: float
    vx: float
    vy: float
    vz: float
    qw: float
    qx: float
    qy: float
    qz: float
    cov_trace: float
    health: str

class ESKFPublisher:

    def __init__(self):
        self._latest_state: ESKFState = self._default_state()
        self._subscribers = []

    def _default_state(self):
        return ESKFState(t=time.time(), px=0.0, py=0.0, pz=0.0, vx=0.0, vy=0.0, vz=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, cov_trace=0.0, health='HEALTHY')

    def subscribe(self, cb):
        self._subscribers.append(cb)

    def publish(self, state: ESKFState):
        self._latest_state = state
        for cb in self._subscribers:
            try:
                cb(state)
            except Exception as e:
                print(f'[ESKF] Subscriber callback error: {e}')

    def latest(self) -> ESKFState:
        return self._latest_state
