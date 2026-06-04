import time
import struct
import pysoem

# this script sets the OP mode to 9 (CSV) and then reads the OP mode from the HEJ if its set correctly

def set_and_verify_mode():
    master = pysoem.Master()
    interface = 'eth0'
    
    print(f"Opening network interface: {interface}...")
    master.open(interface)
    
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found!")
        master.close()
        return

    device = master.slaves[0]
    print(f"Connected to: {device.name}")
    
    # Force the drive to PRE-OP state to unlock configuration mailboxes
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout=50000)

    print("\n[STEP 1] Writing CSV Mode (9) to Modes of Operation (0x6060)...")
    try:
        # 0x6060 is a 1-byte signed integer (b)
        device.sdo_write(0x6060, 0, struct.pack('<b', 9))
        print(" -> SDO Write sent successfully.")
        
        # Give the drive's microchip a brief 100ms window to process the state change
        time.sleep(0.1) 
        
    except Exception as e:
        print(f" -> SDO Write Failed: {e}")
        master.close()
        return

    print("\n[STEP 2] Verifying change via Modes of Operation Display (0x6061)...")
    try:
        # Read back from 0x6061 (1-byte signed integer)
        raw_mode = device.sdo_read(0x6061, 0)
        current_mode = struct.unpack('<b', raw_mode)[0]
        
        print("-" * 50)
        if current_mode == 9:
            print(f"SUCCESS! The drive confirms it has shifted to mode: {current_mode} (CSV)")
        else:
            print(f"NOTICE: The drive returned mode: {current_mode}")
            print("It might require PDO mappings to be loaded before it fully switches displays.")
        print("-" * 50)
        
    except Exception as e:
        print(f" -> SDO Read Failed: {e}")
        
    finally:
        # Gracefully drop the network state and close the port
        print("\nReverting network to safe state and closing...")
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Done.")

if __name__ == "__main__":
    set_and_verify_mode()