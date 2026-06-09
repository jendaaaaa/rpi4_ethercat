import time
import struct
import pysoem

def run_ethercat_csv_harden():
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
    
    # Move to PRE-OP
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout=50000)

    print("Configuring Mode (9) and Velocity Mapping...")
    try:
        device.sdo_write(0x6060, 0, struct.pack('<b', 9))  # CSV Mode
        time.sleep(0.05)
        device.sdo_write(0x1C12, 0, struct.pack('<B', 0))
        device.sdo_write(0x1C13, 0, struct.pack('<B', 0))
        time.sleep(0.05)
        device.sdo_write(0x1C12, 1, struct.pack('<H', 0x1603)) 
        device.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A03)) 
        time.sleep(0.05)
        device.sdo_write(0x1C12, 0, struct.pack('<B', 1))
        device.sdo_write(0x1C13, 0, struct.pack('<B', 1))
    except Exception as e:
        print(f"SDO Setup failed: {e}")
        master.close()
        return

    master.config_map()
    
    # Transition to SAFE-OP
    print("Transitioning to SAFE-OP...")
    master.state = pysoem.SAFEOP_STATE
    master.write_state()
    master.state_check(pysoem.SAFEOP_STATE, timeout=50000)

    print("Transitioning to OP state while feeding the watchdog...")
    master.state = pysoem.OP_STATE
    master.write_state()
    
    # Loop for 2 seconds, continuously sending frames to trick the watchdog
    # and push the state machine into Operational mode
    for _ in range(100):
        master.receive_processdata()
        
        # 0x0080 is the standard CiA 402 command to "Reset Fault" 
        # Sending this clears any latent errors blocking the OP transition
        device.output = struct.pack('<Hi', 0x0080, 0) 
        
        master.send_processdata()
        time.sleep(0.01)
        
        if master.state == pysoem.OP_STATE:
            break

    if master.state_check(pysoem.OP_STATE, timeout=10000) != pysoem.OP_STATE:
        print(f"Stuck! Current actual state code is: {master.state}")
        print("Check if your main motor DC power supply is actually switched on.")
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        return
        
    print("\n>>> SUCCESS! DRIVE IS LIVE IN OP MODE. RUNNING CRAWL... <<<")
    print("-" * 60)

    try:
        boot_sequence = [0x0006, 0x000F]
        target_rpm = 5_000             

        for i in range(300): 
            master.receive_processdata()
            
            if device.input and len(device.input) >= 6:
                status_word, act_position = struct.unpack('<Hi', device.input[:6])
            else:
                status_word, act_position = 0, 0

            if i < 40:
                control_word = boot_sequence[0] # 0x0006
                commanded_speed = 0
            else:
                control_word = boot_sequence[1] # 0x000F
                commanded_speed = target_rpm

            device.output = struct.pack('<Hi', control_word, commanded_speed)
            master.send_processdata()
            
            if i % 30 == 0:
                print(f"Status: {hex(status_word)} | Encoder Position: {act_position}")
                
            time.sleep(0.01)
            
    finally:
        print("\nSafely shutting down...")
        if device.output:
            device.output = struct.pack('<Hi', 0x0000, 0)
            master.send_processdata()
            master.receive_processdata()
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Closed.")

if __name__ == "__main__":
    run_ethercat_csv_harden()