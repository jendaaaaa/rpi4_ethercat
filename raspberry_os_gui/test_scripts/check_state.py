import struct
import pysoem

def check_active_mode():
    master = pysoem.Master()
    interface = 'eth0'
    
    master.open(interface)
    
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found!")
        master.close()
        return

    device = master.slaves[0]
    
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout=50000)

    print("\nReading Modes of Operation Display (Object 0x6061)...")
    try:
        # Object 0x6061, Sub-index 0 is a 1-byte signed integer (SINT)
        raw_mode = device.sdo_read(0x6061, 0)
        mode = struct.unpack('<b', raw_mode)[0]
        
        # specific modes for this HEJ
        mode_names = {
            -60:    "[-60] Cogging Compensation Position Offset Identification Mode (CLOI)",
            -62:    "[-62] Joint Freeze Mode (JF)",
            -64:    "[-64] [DEFAULT] Joint Position Velocity Torque Mode (JPVT)",
            1:      "[1] [FORBIDDEN] Profile Position Mode (PPM)",
            3:      "[3] Profile Velocity Mode (PVM)",
            6:      "[6] Homing Mode",
            8:      "[8] Cyclic Synchronous Position Mode (CSP)",
            9:      "[9] Cyclic Synchronous Velocity Mode (CSV)",
            10:     "[10] Cyclic Synchronous Torque Mode (CST)",
            11:     "[11] Cyclic Synchronous Torque Mode with Commutation Angle (CSTCA) - internal test only"
        }
        
        active_mode_name = mode_names.get(mode, f"Custom/Unknown Mode (Raw Value: {mode})")
        
        print(f"> SUCCESS! Current mode of operation:")
        print(active_mode_name)
        
    except Exception as e:
        print(f"Could not read mode object: {e}")
        print("Tip: If it fails, ensure no other script is running and the drive is powered.")
        
    finally:
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()

if __name__ == "__main__":
    check_active_mode()