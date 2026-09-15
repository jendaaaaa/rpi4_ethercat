"""Poll EPOS4 position objects through SDO without enabling the drive."""

import argparse
import struct
import time

import pysoem


# name, index, subindex, binary format
OBJECTS = (
    ("SSI raw", 0x3012, 0x09, "<I"),
    ("Actual", 0x6064, 0x00, "<i"),
    ("Fused raw", 0x34C6, 0x06, "<i"),
    ("Filtered raw", 0x34C6, 0x0A, "<i"),
    ("Target", 0x607A, 0x00, "<i"),
    ("SW", 0x6041, 0x00, "<H"),
)


def read_object(drive, index, subindex, fmt):
    data = drive.sdo_read(index, subindex)
    return struct.unpack(fmt, data)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", nargs="?", default="eth0")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="seconds between samples (default: 0.5)")
    args = parser.parse_args()
    if not 0.05 <= args.interval <= 60:
        parser.error("interval must be between 0.05 and 60 seconds")

    master = pysoem.Master()
    try:
        master.open(args.interface)
        if master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found")
        drive = master.slaves[0]
        print(f"Slave: {drive.name}", flush=True)
        # config_init discovers slaves and places them in EtherCAT PREOP.
        # No PDO mapping, controlword, target or gain writes are performed.
        if drive.state_check(pysoem.PREOP_STATE, 50_000) != pysoem.PREOP_STATE:
            raise RuntimeError("Drive did not reach PREOP")
        time.sleep(0.5)

        settings = (
            ("Sensor configuration", 0x3000, 0x01, "<I"),
            ("Control structure", 0x3000, 0x02, "<I"),
            ("SSI data bits", 0x3012, 0x02, "<I"),
            ("SSI position bits", 0x3012, 0x0B, "<I"),
            ("SSI power-up time (ms)", 0x3012, 0x08, "<H"),
            ("Position unit", 0x60A8, 0x00, "<I"),
            ("Fusion position unit", 0x34C6, 0x0D, "<I"),
        )
        for name, index, subindex, fmt in settings:
            try:
                value = read_object(drive, index, subindex, fmt)
                print(f"0x{index:04X}:{subindex:02X} {name}: "
                      f"0x{value:08X} ({value})", flush=True)
            except Exception as ex:
                print(f"0x{index:04X}:{subindex:02X} {name}: "
                      f"unavailable: {type(ex).__name__}: {ex!r}", flush=True)

        print("\nAll position columns are raw integer readings. "
              "SSI raw is increments; fused/filtered units are listed above.",
              flush=True)
        print("Move the joint by hand while drive power is disabled. "
              "Ctrl-C stops polling.", flush=True)
        print(f"{'Seconds':>9} " + " ".join(f"{name:>14}" for name, *_ in OBJECTS),
              flush=True)
        started = time.monotonic()
        previous_errors = {}
        while True:
            values = []
            for name, index, subindex, fmt in OBJECTS:
                try:
                    value = read_object(drive, index, subindex, fmt)
                    values.append(f"0x{value:04X}" if name == "SW" else str(value))
                    if name in previous_errors:
                        print(f"{name}: reading recovered", flush=True)
                        del previous_errors[name]
                except Exception as ex:
                    values.append("unavailable")
                    error = f"{type(ex).__name__}: {ex!r}"
                    if previous_errors.get(name) != error:
                        print(f"0x{index:04X}:{subindex:02X} {name}: {error}",
                              flush=True)
                        previous_errors[name] = error
            print(f"{time.monotonic() - started:9.2f} " +
                  " ".join(f"{value:>14}" for value in values), flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    finally:
        master.close()


if __name__ == "__main__":
    main()
