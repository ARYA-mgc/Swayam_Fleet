# dumb json commands. please don't send garbage it will crash the parser
# TODO: migrate to protobuf, json parsing is too slow for 50hz

import json

class SwarmCommand:
    CMD_FOLLOW_ME = 'FOLLOW_ME'
    CMD_LAND = 'LAND'
    CMD_MOVE_TO = 'MOVE_TO'
    CMD_SYNC_TAKEOFF = 'SYNC_TAKEOFF'

    @staticmethod
    def create_command(target_id, cmd_type, params=None):
        return {'type': 'COMMAND', 'target': target_id, 'cmd': cmd_type, 'params': params or {}}

    @staticmethod
    def parse_command(payload):
        try:
            data = json.loads(payload)
            if data.get('type') == 'COMMAND':
                return data
        except:
            return None
        return None
