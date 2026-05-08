# the main loop running on the pi. it's 90% async boilerplate 
# if this crashes the drone falls out of the sky

import time
import asyncio
import json
import socket
from src.swayam.comms.mav import MavlinkBridge
from src.swayam.comms.telem import SwarmTelemetry
from src.swayam.hardware.hw import get_connection_string
from src.swayam.comms.cmds import SwarmCommand
from src.swayam.control.logic import AutonomousSwarmLogic

class SwarmNode:

    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.bridge = MavlinkBridge(get_connection_string())
        self.telemetry = SwarmTelemetry(drone_id)
        self.logic = AutonomousSwarmLogic(drone_id)
        self.running = True
        self.current_mode = 'AUTO_IDLE'
        self._tasks = []

    async def telemetry_broadcast_loop(self):
        while self.running:
            self.telemetry.broadcast_status(12.9716, 77.5946, 10.0, 90, self.current_mode)
            await asyncio.sleep(1)

    async def command_listener_loop(self):
        self.telemetry.sock.bind(('', self.telemetry.port))
        self.telemetry.sock.setblocking(False)
        loop = asyncio.get_event_loop()
        while self.running:
            try:
                data = await loop.sock_recv(self.telemetry.sock, 2048)
                payload = data.decode('utf-8')
                cmd = SwarmCommand.parse_command(payload)
                if cmd and (cmd['target'] == self.drone_id or cmd['target'] == 'ALL'):
                    self.handle_command(cmd)
                else:
                    try:
                        telem_data = json.loads(payload)
                        if 'id' in telem_data:
                            self.logic.update_swarm_state(telem_data['id'], telem_data)
                    except:
                        pass
            except asyncio.CancelledError:
                break
            except BlockingIOError:
                await asyncio.sleep(0.01)
            except Exception as e:
                print(f'[ERR] Listener error: {e}')
                await asyncio.sleep(0.1)

    def handle_command(self, cmd):
        print(f"[NODE] Executing Command: {cmd['cmd']}")
        if cmd['cmd'] == SwarmCommand.CMD_LAND:
            self.bridge.set_mode('LAND')
        elif cmd['cmd'] == SwarmCommand.CMD_FOLLOW_ME:
            self.current_mode = f"FOLLOWING_{cmd['params'].get('leader')}"
        elif cmd['cmd'] == SwarmCommand.CMD_MOVE_TO:
            p = cmd['params']
            self.bridge.send_global_position_int(p['lat'], p['lon'], p['alt'], p['alt'])

    async def autonomous_logic_loop(self):
        while self.running:
            # t0 = time.time()
            try:
                my_pos = (0.0, 0.0, 0.0)  # TODO: get actual position from INS, this is just a placeholder
                my_vel = (0.0, 0.0, 0.0)  # same
                if 'FOLLOWING' in self.current_mode:
                    leader_id = self.current_mode.split('_')[1]
                    target = self.logic.calculate_formation_target(leader_id, offset_n=2.0, offset_e=2.0)
                    if target:
                        print(f'[AUTO] Following {leader_id} -> {target}')
                risk_id, risk_type = self.logic.check_collision_course(my_pos, my_vel)
                if risk_id and risk_type != 'SAFE':
                    print(f'[WARNING] Collision {risk_type} with {risk_id}! Taking evasive action...')
                    target = self.logic.compute_evasion_target(my_pos, risk_id, risk_type)
            except Exception as e:
                print(f'[AUTO ERR] {e}')
            # print(f"logic loop took {(time.time()-t0)*1000}ms")
            await asyncio.sleep(0.5)

    async def main_logic(self):
        print(f'[NODE] Autonomous Node {self.drone_id} online.')
        self._tasks.append(asyncio.create_task(self.telemetry_broadcast_loop()))
        self._tasks.append(asyncio.create_task(self.command_listener_loop()))
        self._tasks.append(asyncio.create_task(self.autonomous_logic_loop()))
        try:
            while self.running:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            self.running = False
            for t in self._tasks:
                t.cancel()
if __name__ == '__main__':
    node = SwarmNode('DRONE_01')
    try:
        asyncio.run(node.main_logic())
    except KeyboardInterrupt:
        pass
