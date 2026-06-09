# sudo taskset -c 3 chrt -f 99 .venv/bin/python3 rpi4_ethercat/raspberry_os_gui/test_scripts/run_jpvt.py

import time
import struct
import pysoem

INTERFACE = "eth0"

CYCLE_S = 0.002
TARGET_VELOCITY_RPM = 15_000

JPVT_MODE = -64
D_GAIN = 10_000

def main():
    master = pysoem.Master()
    master.open(INTERFACE)

    try:
        if master.config_init() <= 0:
            print("No EtherCAT slaves found!")
            return

        drive = master.slaves[0]
        print(f"Found slave: {drive.name}")

        # ------------------------------------------------------------
        # PREOP
        # ------------------------------------------------------------
        master.state = pysoem.PREOP_STATE
        master.write_state()
        master.state_check(pysoem.PREOP_STATE, timeout=50_000)
        print("State: PREOP")

        # JPVT mode
        write_i8(drive, 0x6060, 0, JPVT_MODE)
        time.sleep(0.1)

        mode = read_i8(drive, 0x6061, 0)
        print(f"Mode display 0x6061: {mode}")

        if mode != JPVT_MODE:
            raise RuntimeError("JPVT mode was not accepted.")

        # JPVT gains, UNSIGNED32
        write_u32(drive, 0x34C6, 1, 0)        # P gain
        write_u32(drive, 0x34C6, 2, 0)        # I gain
        write_u32(drive, 0x34C6, 3, D_GAIN)   # D gain

        print(
            "JPVT gains:",
            "P =", read_u32(drive, 0x34C6, 1),
            "I =", read_u32(drive, 0x34C6, 2),
            "D =", read_u32(drive, 0x34C6, 3),
        )

        # Clear feed-forward target joint torque
        try:
            drive.sdo_write(0x34C3, 0, b"\x00\x00\x00\x00")
            print("Target joint torque 0x34C3 cleared")
        except Exception as e:
            print(f"Could not clear 0x34C3, continuing: {e}")

        # Target velocity by SDO, same idea as EPOS Studio
        write_i32(drive, 0x60FF, 0, TARGET_VELOCITY_RPM)
        print(f"Target velocity SDO 0x60FF: {read_i32(drive, 0x60FF, 0)}")

        # 2 ms interpolation period
        drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))

        # ------------------------------------------------------------
        # PDO mapping
        #
        # RxPDO 0x1603:
        #   0x6040:00 Controlword      16 bit
        #   0x60FF:00 Target velocity  32 bit
        #
        # TxPDO 0x1A03:
        #   0x6041:00 Statusword        16 bit
        #   0x34C6:0B Filtered velocity 32 bit
        # ------------------------------------------------------------
        drive.sdo_write(0x1C12, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1C13, 0, struct.pack("<B", 0))

        # RxPDO mapping
        drive.sdo_write(0x1603, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1603, 1, struct.pack("<I", 0x60400010))
        drive.sdo_write(0x1603, 2, struct.pack("<I", 0x60FF0020))
        drive.sdo_write(0x1603, 0, struct.pack("<B", 2))

        # TxPDO mapping: Statusword + filtered JPVT velocity
        drive.sdo_write(0x1A03, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1A03, 1, struct.pack("<I", 0x60410010))
        drive.sdo_write(0x1A03, 2, struct.pack("<I", 0x34C60B20))
        drive.sdo_write(0x1A03, 0, struct.pack("<B", 2))

        # Assign PDOs to Sync Managers
        drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603))
        drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))

        drive.sdo_write(0x1C12, 0, struct.pack("<B", 1))
        drive.sdo_write(0x1C13, 0, struct.pack("<B", 1))

        master.config_map()

        # ------------------------------------------------------------
        # SAFEOP
        # ------------------------------------------------------------
        master.state = pysoem.SAFEOP_STATE
        master.write_state()
        master.state_check(pysoem.SAFEOP_STATE, timeout=50_000)
        print("State: SAFEOP")

        print(f"RxPDO size: {len(drive.output)} bytes, expected 6")
        print(f"TxPDO size: {len(drive.input)} bytes, expected 6")

        # ------------------------------------------------------------
        # OP
        # ------------------------------------------------------------
        master.state = pysoem.OP_STATE
        master.write_state()

        print("Feeding watchdog...")
        for _ in range(300):
            cycle(master, drive, 0x0000, TARGET_VELOCITY_RPM)
            time.sleep(CYCLE_S)

        master.read_state()
        if drive.state != pysoem.OP_STATE:
            raise RuntimeError(f"Could not enter OP state. State: {drive.state}")

        print("State: OP")

        # ------------------------------------------------------------
        # EPOS-like enable sequence
        # ------------------------------------------------------------
        print("Controlword 0x0006")
        for _ in range(250):
            sw, filtered_vel = cycle(master, drive, 0x0006, TARGET_VELOCITY_RPM)
            time.sleep(CYCLE_S)

        print(f"Statusword after 0x0006: {hex(sw)}")

        print("Controlword 0x000F")
        for _ in range(250):
            sw, filtered_vel = cycle(master, drive, 0x000F, TARGET_VELOCITY_RPM)
            time.sleep(CYCLE_S)

        print(f"Statusword after 0x000F: {hex(sw)}")

        if (sw & 0x006F) != 0x0027:
            raise RuntimeError("Drive did not reach Operation Enabled.")

        print(f"\nRunning JPVT at {TARGET_VELOCITY_RPM} RPM\n")

        # ------------------------------------------------------------
        # Run
        # ------------------------------------------------------------
        for i in range(3000):
            sw, filtered_vel = cycle(master, drive, 0x000F, TARGET_VELOCITY_RPM)

            if i % 250 == 0:
                print(
                    f"[{i * 2:5d} ms] "
                    f"SW={hex(sw)} | "
                    f"FilteredVel={filtered_vel}"
                )

            time.sleep(CYCLE_S)

    finally:
        shutdown(master)


def cycle(master, drive, controlword, target_velocity):
    drive.output = struct.pack("<Hi", int(controlword), int(target_velocity))

    master.send_processdata()
    master.receive_processdata()

    if len(drive.input) >= 6:
        return struct.unpack("<Hi", drive.input[:6])

    return 0, 0


def shutdown(master):
    print("\nShutting down...")

    try:
        drive = master.slaves[0]

        for _ in range(250):
            cycle(master, drive, 0x000F, 0)
            time.sleep(CYCLE_S)

        for _ in range(100):
            cycle(master, drive, 0x0006, 0)
            time.sleep(CYCLE_S)

        for _ in range(100):
            cycle(master, drive, 0x0000, 0)
            time.sleep(CYCLE_S)

        master.state = pysoem.INIT_STATE
        master.write_state()

    except Exception as e:
        print(f"Shutdown warning: {e}")

    try:
        master.close()
    except Exception:
        pass

    print("Done.")


def write_i8(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<b", int(value)))

def read_i8(drive, index, subindex):
    return struct.unpack("<b", drive.sdo_read(index, subindex)[:1])[0]

def write_i32(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<i", int(value)))

def read_i32(drive, index, subindex):
    return struct.unpack("<i", drive.sdo_read(index, subindex)[:4])[0]

def write_u32(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<I", int(value)))

def read_u32(drive, index, subindex):
    return struct.unpack("<I", drive.sdo_read(index, subindex)[:4])[0]

if __name__ == "__main__":
    main()