"""
Swayam Pi 4 Swarm Node
Main controller for the Raspberry Pi 4 companion computer.
Integrates MAVLink Bridge and Swarm Telemetry.
"""

import time
import threading
import json
from mavlink_bridge import MavlinkBridge
from swarm_telemetry import SwarmTelemetry
from pi_hardware_config import get_connection_string
from swarm_commands import SwarmCommand
from swarm_autonomous_logic import AutonomousSwarmLogic

class SwarmNode:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.bridge = MavlinkBridge(get_connection_string())
        self.telemetry = SwarmTelemetry(drone_id)
        self.logic = AutonomousSwarmLogic(drone_id)
        self.running = True
        self.current_mode = "AUTO_IDLE"

    def telemetry_broadcast_loop(self):
        """Periodically broadcast status."""
        while self.running:
            # Simulated telemetry data
            self.telemetry.broadcast_status(12.9716, 77.5946, 10.0, 90, self.current_mode)
            time.sleep(1)

    def command_listener_loop(self):
        """Listens for commands and updates swarm state."""
        self.telemetry.sock.bind(('', self.telemetry.port))
        while self.running:
            try:
                data, addr = self.telemetry.sock.recvfrom(2048)
                payload = data.decode('utf-8')
                
                # Check if it's a command
                cmd = SwarmCommand.parse_command(payload)
                if cmd and (cmd['target'] == self.drone_id or cmd['target'] == "ALL"):
                    self.handle_command(cmd)
                else:
                    # Treat as telemetry update
                    try:
                        telem_data = json.loads(payload)
                        if "id" in telem_data:
                            self.logic.update_swarm_state(telem_data['id'], telem_data)
                    except:
                        pass
            except Exception as e:
                print(f"[ERR] Listener error: {e}")

    def handle_command(self, cmd):
        """Executes an incoming command."""
        print(f"[NODE] Executing Command: {cmd['cmd']}")
        if cmd['cmd'] == SwarmCommand.CMD_LAND:
            self.bridge.set_mode("LAND")
        elif cmd['cmd'] == SwarmCommand.CMD_FOLLOW_ME:
            self.current_mode = f"FOLLOWING_{cmd['params'].get('leader')}"
        elif cmd['cmd'] == SwarmCommand.CMD_MOVE_TO:
            p = cmd['params']
            self.bridge.send_global_position_int(p['lat'], p['lon'], p['alt'], p['alt'])

    def autonomous_logic_loop(self):
        """Runs the autonomous decision making logic."""
        while self.running:
            if "FOLLOWING" in self.current_mode:
                leader_id = self.current_mode.split("_")[1]
                target = self.logic.calculate_follow_offset(leader_id)
                if target:
                    print(f"[AUTO] Following {leader_id} -> {target}")
                    self.bridge.send_global_position_int(target[0], target[1], target[2], target[2])
            
            # Simple collision avoidance check
            risk = self.logic.check_collision_risk((12.9716, 77.5946))
            if risk:
                print(f"[WARNING] Collision risk with {risk}! Taking evasive action...")
                # Evasive logic (e.g., climb 2m)
                self.bridge.send_global_position_int(12.9716, 77.5946, 12.0, 12.0)
            
            time.sleep(0.5)

    def main_logic(self):
        """Handle missions and swarm logic."""
        print(f"[NODE] Autonomous Node {self.drone_id} online.")
        
        # Start loops
        threading.Thread(target=self.telemetry_broadcast_loop, daemon=True).start()
        threading.Thread(target=self.command_listener_loop, daemon=True).start()
        threading.Thread(target=self.autonomous_logic_loop, daemon=True).start()

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False

if __name__ == "__main__":
    node = SwarmNode("DRONE_01")
    node.main_logic()
