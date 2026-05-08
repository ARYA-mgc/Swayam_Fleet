# checks if the raspberry pi is melting
# thermal throttling is a feature not a bug

import os
import psutil
import time

class SystemHealth:

    def __init__(self, bridge):
        self.bridge = bridge

    def get_pi_stats(self):
        stats = {'cpu_usage': psutil.cpu_percent(), 'ram_usage': psutil.virtual_memory().percent, 'temp': self._get_temp()}
        # if stats['temp'] > 80: print("cpu is melting:", stats['temp'])
        return stats

    def _get_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return float(f.read()) / 1000.0
        except:
            return 0.0

    def get_pixhawk_health(self):
        return {'connected': self.bridge.master.target_system > 0, 'last_heartbeat': time.time() - self.bridge.master.last_heartbeat}

    def report_health(self):
        report = {'pi': self.get_pi_stats(), 'pixhawk': self.get_pixhawk_health(), 'timestamp': time.time()}
        return report
if __name__ == '__main__':
    pass
