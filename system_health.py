"""
Swayam System Health Monitor
Monitors RPi 4 resources and Pixhawk status for the swarm.
"""

import os
import psutil
import time

class SystemHealth:
    def __init__(self, bridge):
        self.bridge = bridge

    def get_pi_stats(self):
        """Returns CPU, RAM, and Temperature of the RPi 4."""
        stats = {
            "cpu_usage": psutil.cpu_percent(),
            "ram_usage": psutil.virtual_memory().percent,
            "temp": self._get_temp()
        }
        return stats

    def _get_temp(self):
        """Gets CPU temperature (Linux/RPi specific)."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return float(f.read()) / 1000.0
        except:
            return 0.0

    def get_pixhawk_health(self):
        """Checks Pixhawk heartbeat and basic status."""
        # This would typically involve checking heartbeat and system status messages
        return {
            "connected": self.bridge.master.target_system > 0,
            "last_heartbeat": time.time() - self.bridge.master.last_heartbeat
        }

    def report_health(self):
        """Combines all stats into a health report."""
        report = {
            "pi": self.get_pi_stats(),
            "pixhawk": self.get_pixhawk_health(),
            "timestamp": time.time()
        }
        return report

if __name__ == "__main__":
    # Note: Requires 'psutil' (pip install psutil)
    pass
