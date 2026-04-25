"""
Swayam Mission Planner Config
=============================
Generates connection strings and helper tips for Mission Planner.
"""

def get_mp_connection_info(num_drones=3, base_port=14550):
    print("\n" + "="*50)
    print(" SWAYAM - MISSION PLANNER CONNECTION GUIDE")
    print("="*50)
    print(f"\nTarget GCS Port: {base_port}")
    print("\nHow to connect:")
    print(f"1. Open Mission Planner.")
    print(f"2. Select 'UDP' from the top-right dropdown.")
    print(f"3. Click 'Connect'.")
    print(f"4. Enter port: {base_port}")
    print("\nDetected Swarm Configuration:")
    for i in range(1, num_drones + 1):
        print(f"  - Drone {i}: System ID {i}")
    print("\nAdvanced: If running multiple GCS instances, use:")
    for i in range(num_drones):
        print(f"  - GCS {i+1}: 127.0.0.1:{base_port + i}")
    print("="*50 + "\n")

if __name__ == "__main__":
    get_mp_connection_info()
