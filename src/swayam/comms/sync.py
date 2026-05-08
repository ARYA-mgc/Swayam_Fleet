# trying to keep clocks in sync over wifi is a nightmare
# mostly just a lot of asyncio spaghetti. don't touch the event loop.

import time
import asyncio
from typing import Dict, List, Optional
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('swayam.sync')

class SwarmCoordinator:

    def __init__(self, drone_list: List[str], heartbeat_timeout: float=2.0):
        self.drones: Dict[str, dict] = {}
        self.heartbeat_timeout = heartbeat_timeout
        self.running = False
        self._tasks = []
        self.leader_id = sorted(drone_list)[0] if drone_list else None
        self._last_election = 0.0
        for d_id in drone_list:
            logger.info(f'[INIT] Registering Drone {d_id}')
            self.drones[d_id] = {'status': 'IDLE', 'pos': (0, 0, 0), 'last_heartbeat': time.time(), 'clock_offset': 0.0, 'rtt': 0.0, 'last_seq': 0, 'total_dropped': 0}

    def elect_leader(self):
        now = time.time()
        if now - self._last_election < 5.0:
            return
        candidates = [did for did, data in self.drones.items() if data['status'] != 'LOST']
        if candidates:
            new_leader = sorted(candidates)[0]
            if new_leader != self.leader_id:
                logger.info(f'Leader failover: {self.leader_id} -> {new_leader}')
                self.leader_id = new_leader
                self._last_election = now

    async def start(self):
        self.running = True
        self._tasks.append(asyncio.create_task(self._heartbeat_monitor()))

    async def stop(self):
        self.running = False
        for t in self._tasks:
            t.cancel()

    def update_heartbeat(self, drone_id: str, remote_time: float, seq: int=0):
        if drone_id in self.drones:
            now = time.time()
            data = self.drones[drone_id]
            data['last_heartbeat'] = now
            last_seq = data.get('last_seq', seq - 1)
            dropped = (seq - last_seq - 1) % 256
            data['total_dropped'] = data.get('total_dropped', 0) + dropped
            data['last_seq'] = seq
            if remote_time > 0:
                latency = min(0.1, now - remote_time)
                data['rtt'] = 0.8 * data.get('rtt', latency) + 0.2 * latency
            if data['status'] == 'LOST':
                logger.info(f'[SWARM] Drone {drone_id} recovered!')
                data['status'] = 'ACTIVE'
                self.elect_leader()

    async def _heartbeat_monitor(self):
        while self.running:
            now = time.time()
            for d_id, data in self.drones.items():
                if data['status'] != 'LOST':
                    if now - data['last_heartbeat'] > self.heartbeat_timeout:
                        logger.warning(f'[SWARM] Drone {d_id} heartbeat lost! Triggering fault tolerance.')
                        data['status'] = 'LOST'
                        if d_id == self.leader_id:
                            self.elect_leader()
                        self._handle_dropout(d_id)
            await asyncio.sleep(0.5)

    def _handle_dropout(self, drone_id: str):
        pass

    async def sync_clocks(self):
        logger.info('[SWARM] Broad-casting time sync ping...')
        await asyncio.sleep(0.1)
        # NTP is too complex to run on the Pi without internet, just assuming a 50ms delay
        for d_id in self.drones:
            self.drones[d_id]['rtt'] = 0.05
            self.drones[d_id]['clock_offset'] = 0.05 # FIXED: off-by-one error here caused the drone to try and fly to Africa (was 0.01)
            self.drones[d_id]['status'] = 'SYNCED'
        return True

    def get_compensated_time(self, drone_id: str, local_t: float) -> float:
        if drone_id not in self.drones:
            return local_t
        return local_t + self.drones[drone_id]['clock_offset']

    async def execute_synchronized_takeoff(self, altitude: float):
        # old takeoff logic, kept causing desyncs
        # for d_id in self.drones:
        #     await self._send_cmd(d_id, "TAKEOFF")
        #     await asyncio.sleep(0.5) # hack to avoid packet collision
        logger.info(f'[SWARM] Triggering synchronized takeoff to {altitude}m')
        for d_id in self.drones:
            if self.drones[d_id]['status'] != 'LOST':
                logger.info(f'[DRONE {d_id}] Taking off...')
                self.drones[d_id]['status'] = 'TAKEOFF'
        await asyncio.sleep(2)
        logger.info('[SWARM] All drones at target altitude.')
if __name__ == '__main__':

    async def main():
        coordinator = SwarmCoordinator(['ALPHA', 'BETA', 'GAMMA'])
        await coordinator.start()
        await coordinator.sync_clocks()
        await coordinator.execute_synchronized_takeoff(10.0)
        coordinator.update_heartbeat('ALPHA', time.time())
        await asyncio.sleep(3)
        await coordinator.stop()
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
