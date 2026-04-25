"""
Swayam Raspberry Pi 4 Hardware Configuration
Settings for Serial and GPIO interface with Pixhawk Cube Orange.
"""

# Serial Port Configurations
# RPi 4 often uses /dev/ttyAMA0 for primary UART or /dev/ttyS0
PI_SERIAL_PORT = "/dev/ttyAMA0"
PI_BAUD_RATE = 115200

# Pixhawk Connection Config
CUBE_ORANGE_SYSID = 1
SWARM_SYSID_START = 10

# GPIO Pins for Status LEDs (Example)
GPIO_LED_LINK = 17    # Green LED for MAVLink Link
GPIO_LED_ERR = 27     # Red LED for Errors
GPIO_LED_SWARM = 22   # Blue LED for Swarm Sync Status

# Power Management
VOLTAGE_THRESHOLD = 11.1  # 3S LiPo safety cutoff

def get_connection_string():
    """Returns the serial connection string for MAVLink."""
    return f"{PI_SERIAL_PORT},{PI_BAUD_RATE}"
