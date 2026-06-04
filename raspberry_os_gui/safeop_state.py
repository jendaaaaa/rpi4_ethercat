import time
import struct
import pysoem

def run_ethercat_test():
    # initialize master
    master = pysoem.Master()
    interface = 'eth0'
    
    print(f"Opening network interface: {interface}...")
    master.open(interface)
    
    # scan network for slaves
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found! Make sure the drive is powered on and connected.")
        master.close()
        return

    print(f"Success! Found {len(master.slaves)} slave(s).")
    device = master.slaves[0]
    print(f"Slave Name: {device.name}")
    
    # read SDO
    try:
        vendor_id = struct.unpack('<I', device.sdo_read(0x1018, 1))[0]
        product_code = struct.unpack('<I', device.sdo_read(0x1018, 2))[0]
        print(f"  -> Vendor ID: {hex(vendor_id)}")
        print(f"  -> Product Code: {hex(product_code)}")
    except Exception as e:
        print(f"Could not read SDO info: {e}")

    # map PDO
    master.config_map()

    # move to SAFE-OP state
    print("Transitioning to Safe-Operational state...")
    if master.state_check(pysoem.SAFEOP_STATE, timeout=50000) != pysoem.SAFEOP_STATE:
        print("Failed to reach Safe-Operational State.")
        master.close()
        return
        
    print("Drive is now in SAFE-OPERATIONAL mode. Reading 5 seconds of live data...")
    print("-" * 60)
    print(f"{'Iteration':<10} | {'Statusword':<12} | {'Actual Position':<15}")
    print("-" * 60)

    try:
        for i in range(50):
            # request process data
            master.receive_processdata()
            
            # unpack raw bytes
            # 2 bytes (H = unsigned short) for STATUSWORD, 4 bytes (i = int) for POSITION
            if device.input:
                print(f"Iteration {i+1}: Received {len(device.input)} bytes of data from the HEJ.")
                print(f"Raw hex: {device.input.hex()}")
            else:
                print(f"Iteration {i+1:<9}: Input buffer is empty.")

            # send empty frame to keep connection alive
            master.send_processdata()
            
            time.sleep(0.1)
            
    finally:
        # shutdown
        print("-" * 60)
        print("Reverting drive back to Init State and closing connection...")
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Network closed successfully.")

if __name__ == "__main__":
    run_ethercat_test()