# sudo taskset -c 3 chrt -f 99 .venv/bin/python3 rpi4_ethercat/raspberry_os_gui/test_scripts/run_jpvt.py

import time
import struct
import pysoem

INTERFACE = "eth0"

CYCLE_S = 0.002
TARGET_VELOCITY_RPM = 0
POSITION_OFFSET = -500  # Signed increments (inc); added after converting startup feedback
N_CYCLES = 2_000

JPVT_MODE = -64

P_GAIN = 50_000
I_GAIN = 0
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
        if drive.state_check(pysoem.PREOP_STATE, timeout=50_000) != pysoem.PREOP_STATE:
            raise RuntimeError("Drive did not reach PREOP for startup recovery.")
        print("State: PREOP")
        reset_startup_fault(drive)

        # JPVT mode
        write_i8(drive, 0x6060, 0, JPVT_MODE)
        time.sleep(0.1)

        mode = read_i8(drive, 0x6061, 0)
        print(f"Mode display 0x6061: {mode}")

        if mode != JPVT_MODE:
            raise RuntimeError("JPVT mode was not accepted.")

        # JPVT gains, UNSIGNED32
        write_u32(drive, 0x34C6, 1, P_GAIN)   # P gain
        write_u32(drive, 0x34C6, 2, I_GAIN)   # I gain
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

        # Initialize target velocity; also sent in every command PDO.
        write_i32(drive, 0x60FF, 0, TARGET_VELOCITY_RPM)
        print(f"Target velocity SDO 0x60FF: {read_i32(drive, 0x60FF, 0)}")

        # Check configured units; do not change the drive's unit settings.
        fusion_units_per_inc = read_position_scale(drive)
        target_position = round(read_i32(drive, 0x34C6, 6) / fusion_units_per_inc)
        write_i32(drive, 0x607A, 0, target_position)

        # Anti-alias cutoff = half the configured EtherCAT cycle frequency.
        # 0x3676:01 returned a 2-byte value of 200 for the documented 200 Hz default.
        cutoff_hz = round(1.0 / (2.0 * CYCLE_S))
        if not 1 <= cutoff_hz <= 0xFFFF:
            raise ValueError("Anti-alias cutoff must fit a positive 16-bit value.")
        drive.sdo_write(0x3676, 1, struct.pack("<H", cutoff_hz))
        actual_cutoff = struct.unpack("<H", drive.sdo_read(0x3676, 1))[0]
        if actual_cutoff != cutoff_hz:
            raise RuntimeError(
                f"Anti-alias cutoff readback mismatch: {actual_cutoff}, expected {cutoff_hz}"
            )
        print(f"Anti-alias cutoff 0x3676:01: {actual_cutoff} Hz")

        # 2 ms interpolation period
        drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))

        # ------------------------------------------------------------
        # PDO mapping
        #
        # RxPDO 0x1603:
        #   0x6040:00 Controlword      16 bit
        #   0x60FF:00 Target velocity  32 bit
        #   0x607A:00 Target position  32 bit
        #
        # TxPDO 0x1A03:
        #   0x6041:00 Statusword        16 bit
        #   0x34C6:0B Filtered velocity 32 bit
        #   0x34C6:0A Filtered position 32 bit
        # ------------------------------------------------------------
        drive.sdo_write(0x1C12, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1C13, 0, struct.pack("<B", 0))

        # RxPDO mapping
        drive.sdo_write(0x1603, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1603, 1, struct.pack("<I", 0x60400010))
        drive.sdo_write(0x1603, 2, struct.pack("<I", 0x60FF0020))
        drive.sdo_write(0x1603, 3, struct.pack("<I", 0x607A0020))
        drive.sdo_write(0x1603, 0, struct.pack("<B", 3))

        # TxPDO mapping: Statusword + filtered JPVT velocity + filtered position
        drive.sdo_write(0x1A03, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1A03, 1, struct.pack("<I", 0x60410010))
        drive.sdo_write(0x1A03, 2, struct.pack("<I", 0x34C60B20))
        drive.sdo_write(0x1A03, 3, struct.pack("<I", 0x34C60A20))
        drive.sdo_write(0x1A03, 0, struct.pack("<B", 3))

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

        print(f"RxPDO size: {len(drive.output)} bytes, expected 10")
        print(f"TxPDO size: {len(drive.input)} bytes, expected 10")

        # ------------------------------------------------------------
        # OP
        # ------------------------------------------------------------
        master.state = pysoem.OP_STATE
        master.write_state()

        print("Feeding watchdog...")
        for _ in range(300):
            cycle(master, drive, 0x0000, TARGET_VELOCITY_RPM, target_position)
            time.sleep(CYCLE_S)

        master.read_state()
        if drive.state != pysoem.OP_STATE:
            raise RuntimeError(f"Could not enter OP state. State: {drive.state}")

        print("State: OP")

        # Read once before enabling, then apply a fixed relative position step.
        start_position_raw = read_i32(drive, 0x34C6, 6)
        start_position = round(start_position_raw / fusion_units_per_inc)
        if int(POSITION_OFFSET) != POSITION_OFFSET:
            raise ValueError("POSITION_OFFSET must be a whole number of increments.")
        target_position = start_position + int(POSITION_OFFSET)
        if not -(1 << 31) <= target_position < (1 << 31):
            raise ValueError("Startup position + offset exceeds signed 32-bit range.")
        print(
            f"StartPos={start_position} inc | Offset={POSITION_OFFSET} inc | "
            f"TargetPos={target_position}"
        )

        # ------------------------------------------------------------
        # EPOS-like enable sequence
        # ------------------------------------------------------------
        for controlword, expected_state in (
            (0x0006, 0x0021),  # Ready to switch on
            (0x0007, 0x0023),  # Switched on
            (0x000F, 0x0027),  # Operation enabled
        ):
            print(f"Controlword {controlword:#06x}")
            deadline = time.monotonic() + 1.0
            while True:
                sw, filtered_vel, filtered_pos = cycle(
                    master, drive, controlword, TARGET_VELOCITY_RPM, target_position
                )
                if sw & 0x0008:
                    raise RuntimeError(f"Drive fault during enable: SW={sw:#06x}")
                if (sw & 0x006F) == expected_state:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"Enable timed out: CW={controlword:#06x}, SW={sw:#06x}"
                    )
                time.sleep(CYCLE_S)
            print(f"Statusword: {sw:#06x}")

        print(f"\nRunning JPVT with target position {target_position}\n")

        # ------------------------------------------------------------
        # Run
        # ------------------------------------------------------------
        for i in range(N_CYCLES):
            sw, filtered_vel, filtered_pos = cycle(master, drive, 0x000F, TARGET_VELOCITY_RPM, target_position)

            if sw & 0x0008:
                raise RuntimeError(f"Drive fault while running: SW={sw:#06x}")

            if i % 250 == 0:
                print(
                    f"[{i * CYCLE_S * 1000:7.0f} ms] "
                    f"SW=0x{sw:04X} | "
                    f"Pos={filtered_pos / fusion_units_per_inc:13.3f} inc | "
                    f"Target={target_position:11d} inc | "
                    f"Vel(raw)={filtered_vel:11d}"
                )

            time.sleep(CYCLE_S)

    finally:
        shutdown(master)


def read_position_scale(drive):
    target_unit = read_u32(drive, 0x60A8, 0)
    fusion_unit = read_u32(drive, 0x34C6, 0x0D)
    if target_unit != 0x00B50000:
        raise RuntimeError(f"Unsupported target position unit: {target_unit:#010x}")
    scales = {0x00B50000: 1, 0xFDB50000: 1000}
    if fusion_unit not in scales:
        raise RuntimeError(f"Unsupported sensor-fusion position unit: {fusion_unit:#010x}")
    scale = scales[fusion_unit]
    print(
        f"Position units: target=inc; fusion={fusion_unit:#010x}; "
        f"{scale} feedback units per increment"
    )
    return scale


def reset_startup_fault(drive):
    sw = struct.unpack("<H", drive.sdo_read(0x6041, 0))[0]
    err = struct.unpack("<B", drive.sdo_read(0x1001, 0))[0]
    print(f"Startup statusword: {sw:#06x}; error register 0x1001: {err:#04x}")
    print("Error history [0x1003]:")
    try:
        count = struct.unpack("<B", drive.sdo_read(0x1003, 0))[0]
        print(f"  {count} errors stored")
        for i in range(1, min(count, 5) + 1):
            error = read_u32(drive, 0x1003, i)
            print(f"  [{i}]: {error:#010x}")
    except Exception as ex:
        print(f"  Could not read history: {ex}")

    # Leave the drive disabled during configuration; also lower reset bit 7.
    drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
    if not (sw & 0x0008):
        print("No startup fault; reset skipped.")
        return

    # A fault reaction must finish before the fault can be acknowledged.
    deadline = time.monotonic() + 1.0
    while (sw & 0x004F) != 0x0008:
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Fault reaction did not finish: SW={sw:#06x}")
        time.sleep(0.02)
        sw = struct.unpack("<H", drive.sdo_read(0x6041, 0))[0]

    print("Sending one startup fault-reset pulse (CW bit 7)...")
    time.sleep(0.02)
    try:
        drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))
        time.sleep(0.2)
    finally:
        drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))

    deadline = time.monotonic() + 1.0
    while True:
        sw = struct.unpack("<H", drive.sdo_read(0x6041, 0))[0]
        if not (sw & 0x0008):
            print(f"Statusword after reset: {sw:#06x}")
            return
        if time.monotonic() >= deadline:
            err = struct.unpack("<B", drive.sdo_read(0x1001, 0))[0]
            raise RuntimeError(
                f"Startup fault persists: SW={sw:#06x}, error register={err:#04x}"
            )
        time.sleep(0.02)


def cycle(master, drive, controlword, target_velocity, target_position):
    drive.output = struct.pack("<Hii", int(controlword), int(target_velocity), int(target_position))

    master.send_processdata()
    master.receive_processdata()

    if len(drive.input) >= 10:
        return struct.unpack("<Hii", drive.input[:10])

    return 0, 0, 0


def shutdown(master):
    print("\nShutting down...")

    try:
        drive = master.slaves[0]

        # Recovery can fail before the 10-byte command PDO is mapped.
        if len(drive.output) != 10:
            drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
            return

        # Keep the last position command while disabling; do not command zero.
        _, _, target_position = struct.unpack("<Hii", drive.output[:10])

        for _ in range(100):
            cycle(master, drive, 0x0006, 0, target_position)
            time.sleep(CYCLE_S)

        for _ in range(100):
            cycle(master, drive, 0x0000, 0, target_position)
            time.sleep(CYCLE_S)

    except Exception as e:
        print(f"Shutdown warning: {e}")
    finally:
        try:
            master.state = pysoem.INIT_STATE
            master.write_state()
        except Exception as e:
            print(f"INIT warning: {e}")
        try:
            master.close()
        except Exception as e:
            print(e)
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