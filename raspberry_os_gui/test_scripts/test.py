import pysoem
import sys

NETWORK_IFACE = 'eth0' 

def test_connection():
    master = pysoem.Master()
    
    try:
        master.open(NETWORK_IFACE)
        print(f"Successfully opened {NETWORK_IFACE}")
    except Exception as e:
        print(f"Failed to open interface: {e}")
        sys.exit(1)

    # config_init() scans the network for slaves
    if master.config_init() > 0:
        print(f"\nSuccess! Found {len(master.slaves)} EtherCAT slave(s):")
        for i, slave in enumerate(master.slaves):
            print(f"  [{i+1}] {slave.name}")
    else:
        print("\nNo slaves found. Check your ethernet cable and ensure the Maxon drive is powered on.")

    master.close()

if __name__ == '__main__':
    test_connection()