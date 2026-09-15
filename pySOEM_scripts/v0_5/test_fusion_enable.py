"""Compare joint feedback before/after enabling JPVT with zero command gains."""

import argparse
from pathlib import Path
import struct
import sys
import time

import pysoem

# Reuse the working version-2 controller; no source files are modified.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "v0_3"))
from controller import CYCLE_S, JPVTController, log


class FusionTestController(JPVTController):
    def __init__(self, interface):
        super().__init__(interface)
        # Set these BEFORE configure() writes SDOs or sends any PDOs.
        self.kp = self.ki = self.kd = 0
        self.target_velocity = self.target_torque = 0
        self.extra_feedback = (0, 0)
        self.require_full_wkc = False

    def _configure_pdos(self):
        super()._configure_pdos()
        # Read all diagnostic positions in the SAME process-data exchange.
        # SW, velocity, filtered position, torque, fused position, actual.
        mappings = (
            0x60410010, 0x34C60B20, 0x34C60A20, 0x36770020,
            0x34C60620, 0x60640020,
        )
        self.drive.sdo_write(0x1C13, 0, b"\x00")
        self.drive.sdo_write(0x1A03, 0, b"\x00")
        for subindex, mapping in enumerate(mappings, 1):
            self.drive.sdo_write(0x1A03, subindex, struct.pack("<I", mapping))
        self.drive.sdo_write(0x1A03, 0, struct.pack("<B", len(mappings)))
        self.drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))
        self.drive.sdo_write(0x1C13, 0, b"\x01")

    def configure(self):
        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found")
        self.drive = self.master.slaves[0]
        log(f"Found slave: {self.drive.name}")
        self.master.state = pysoem.PREOP_STATE
        self.master.write_state()
        if self.drive.state_check(pysoem.PREOP_STATE, 50_000) != pysoem.PREOP_STATE:
            raise RuntimeError("Drive did not reach PREOP")
        self._reset_fault_if_needed()
        self._configure_jpvt()
        self._configure_pdos()
        self.master.config_map()
        if len(self.drive.output) != 26 or len(self.drive.input) != 22:
            raise RuntimeError("Expected 26-byte RxPDO and 22-byte test TxPDO")
        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        if self.master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
            raise RuntimeError("Master did not reach SAFEOP")
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        for _ in range(300):
            self.cycle(0x0000)
            time.sleep(CYCLE_S)
        self.master.read_state()
        if self.drive.state != pysoem.OP_STATE:
            raise RuntimeError(
                f"Drive did not reach OP: state=0x{self.drive.state:02X}, "
                f"AL status=0x{self.drive.al_status:04X}"
            )
        self.require_full_wkc = True
        self.cycle(0x0000)
        log("Ready: P=I=D=0, velocity=torque=0. Test never restores nonzero gains.")

    def cycle(self, controlword):
        self.drive.output = struct.pack(
            "<HiiiIII", controlword, 0, self.target_position, 0, 0, 0, 0
        )
        self.master.send_processdata()
        wkc = self.master.receive_processdata()
        # SAFEOP may return input data without accepting output data yet.
        # Require complete exchange only after OP has been confirmed.
        if wkc <= 0 or (self.require_full_wkc and wkc != self.master.expected_wkc):
            raise RuntimeError(f"WKC={wkc}, expected {self.master.expected_wkc}")
        if len(self.drive.input) != 22:
            raise RuntimeError("Expected 22 input bytes")
        sw, velocity, filtered, torque, fused, actual = struct.unpack(
            "<Hiiiii", self.drive.input
        )
        self.feedback = (sw, velocity, filtered, torque)
        self.extra_feedback = (fused, actual)
        return self.feedback


def poll(controller, phase, duration, started):
    deadline = time.monotonic() + duration
    next_print = 0
    while time.monotonic() < deadline:
        cycle_start = time.monotonic()
        controller.run_cycle()
        sw, velocity, filtered, torque = controller.feedback
        if sw & 0x0008:
            raise RuntimeError(f"Drive fault: SW=0x{sw:04X}")
        if cycle_start >= next_print:
            fused, actual = controller.extra_feedback
            scale = controller.position_scale
            print(f"{cycle_start-started:8.2f} {phase:>9} "
                  f"0x{sw:04X} {actual:12d} {fused/scale:12.3f} "
                  f"{filtered/scale:12.3f} {controller.target_position:12d} "
                  f"{torque:10d} {velocity:10d}", flush=True)
            next_print = cycle_start + 0.5
        remaining = CYCLE_S - (time.monotonic() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", nargs="?", default="eth0")
    parser.add_argument("--seconds", type=float, default=20,
                        help="enabled observation duration (default: 20 seconds)")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error("seconds must be between 1 and 300")
    controller = FusionTestController(args.interface)
    try:
        controller.configure()
        print("Seconds     Phase     SW       Actual      FusedInc  FilteredInc "
              "   TargetInc  Torque_mNm    VelRaw", flush=True)
        started = time.monotonic()
        poll(controller, "disabled", 5, started)
        log("Enabling now with ZERO P/I/D, torque and velocity.")
        controller.pre_enable()
        poll(controller, "pre-enabled", args.seconds, started)
        controller.disable()
        poll(controller, "disabled", 5, started)
    except KeyboardInterrupt:
        log("Interrupted")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
