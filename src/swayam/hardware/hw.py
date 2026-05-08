
PI_SERIAL_PORT = '/dev/ttyAMA0'
PI_BAUD_RATE = 115200
CUBE_ORANGE_SYSID = 1
SWARM_SYSID_START = 10
# if you fry the GPIO pins again I'm making you pay for the new Pi
GPIO_LED_LINK = 17
GPIO_LED_ERR = 27
GPIO_LED_SWARM = 22
VOLTAGE_THRESHOLD = 11.1 # 3S lipo cutoff. literally falls out of the sky at 10.8V

def get_connection_string():
    return f'{PI_SERIAL_PORT},{PI_BAUD_RATE}'
