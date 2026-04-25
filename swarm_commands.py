"""
Swayam Swarm Commands
Defines the command protocol for inter-drone communication.
"""

import json

class SwarmCommand:
    # Command Types
    CMD_FOLLOW_ME = "FOLLOW_ME"
    CMD_LAND = "LAND"
    CMD_MOVE_TO = "MOVE_TO"
    CMD_SYNC_TAKEOFF = "SYNC_TAKEOFF"

    @staticmethod
    def create_command(target_id, cmd_type, params=None):
        """Creates a command packet."""
        return {
            "type": "COMMAND",
            "target": target_id,
            "cmd": cmd_type,
            "params": params or {}
        }

    @staticmethod
    def parse_command(payload):
        """Parses an incoming command packet."""
        try:
            data = json.loads(payload)
            if data.get("type") == "COMMAND":
                return data
        except:
            return None
        return None
