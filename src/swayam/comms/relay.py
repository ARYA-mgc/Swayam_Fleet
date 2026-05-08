# shoves 5 drones worth of telemetry into one mission planner port
# rate limiting is held together by duct tape and prayers

import asyncio
import time
import logging
from pymavlink import mavutil
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('swayam.relay')

class SwarmGCSRelay:

    def __init__(self, gcs_url='udpout:127.0.0.1:14550', rate_limit_hz=50):
        self.gcs_url = gcs_url
        self.rate_limit_hz = float(rate_limit_hz)
        self._target_rate = float(rate_limit_hz)
        self._alpha = 0.1
        self._delta_max = 5.0
        self.drones = {}
        self.drone_conns = {}
        self.gcs_conn = None
        self._running = False
        self._tasks = []
        self._tokens = rate_limit_hz
        self._last_token_time = time.time()

    def add_drone(self, sysid, connection_string):
        self.drones[sysid] = connection_string
        logger.info(f'Relay: Registered Drone {sysid} at {connection_string}')

    async def start(self):
        self._running = True
        logger.info(f'Relay: Starting GCS bridge at {self.gcs_url}')
        self.gcs_conn = mavutil.mavlink_connection(self.gcs_url, source_system=255)
        self._tasks.append(asyncio.create_task(self._gcs_listener()))
        for sysid, conn_str in self.drones.items():
            self._tasks.append(asyncio.create_task(self._drone_listener(sysid, conn_str)))

    def adjust_backpressure(self, load_factor: float):
        # if we drop below 10hz Mission Planner starts screaming at us
        base_rate = 50.0
        new_target = base_rate / max(0.5, load_factor)
        new_target = max(10.0, min(100.0, new_target))
        ema = (1.0 - self._alpha) * self._target_rate + self._alpha * new_target
        delta = max(-self._delta_max, min(self._delta_max, ema - self._target_rate))
        self._target_rate += delta
        self.rate_limit_hz = self._target_rate

    def _consume_token(self):
        now = time.time()
        elapsed = now - self._last_token_time
        self._tokens = min(self.rate_limit_hz, self._tokens + elapsed * self.rate_limit_hz)
        self._last_token_time = now
        if self._tokens >= 1.0:
            self._tokens -= 1.0
            return True
        return False

    async def _drone_listener(self, sysid, conn_str):
        logger.info(f'Relay: Listening to Drone {sysid}...')
        conn = mavutil.mavlink_connection(conn_str)
        self.drone_conns[sysid] = conn
        while self._running:
            try:
                msg = conn.recv_match(blocking=False)
                if msg:
                    if self._consume_token():
                        self.gcs_conn.mav.send(msg)
                else:
                    await asyncio.sleep(0.01)
            except Exception as e:
                logger.error(f'Relay: Drone {sysid} error: {e}')
                await asyncio.sleep(1)

    async def _gcs_listener(self):
        while self._running:
            try:
                msg = self.gcs_conn.recv_match(blocking=False)
                if msg:
                    target_sys = getattr(msg, 'target_system', 0)
                    if target_sys == 0:
                        for conn in self.drone_conns.values():
                            conn.mav.send(msg)
                    elif target_sys in self.drone_conns:
                        self.drone_conns[target_sys].mav.send(msg)
                else:
                    await asyncio.sleep(0.01)
            except Exception as e:
                logger.error(f'Relay: GCS error: {e}')
                await asyncio.sleep(1)

    async def stop(self):
        self._running = False
        for t in self._tasks:
            t.cancel()
        if self.gcs_conn:
            self.gcs_conn.close()
        for conn in self.drone_conns.values():
            conn.close()
if __name__ == '__main__':

    async def main():
        relay = SwarmGCSRelay('udpout:127.0.0.1:14550')
        relay.add_drone(1, 'udpin:127.0.0.1:14551')
        relay.add_drone(2, 'udpin:127.0.0.1:14552')
        await relay.start()
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            await relay.stop()
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
