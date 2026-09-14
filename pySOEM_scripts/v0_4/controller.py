"""EPOS4 JPVT controller used by the Unitree-compatible DDS bridge."""

import struct
import math
import sys
import time

import pysoem

INTERFACE = "eth0"
CYCLE_S = 0.002
JPVT_MODE = -64
P_GAIN = 50_000
I_GAIN = 0
D_GAIN = 10_000
POSITION_INCREMENTS_PER_REV = 4096  # HW8A04 documented default (12-bit SSI)


def log(message):
    print(message, file=sys.stderr, flush=True)


class JPVTController:
    def __init__(self, interface):
        self.master = pysoem.Master()
        self.master.open(interface)
        self.drive = None
        self.position_scale = 1
        self.target_position = 0
        self.target_velocity = 0
        self.target_torque = 0
        self.p_gain = P_GAIN
        self.i_gain = I_GAIN
        self.d_gain = D_GAIN
        self.feedback = (0, 0, 0, 0)  # status, velocity, position, torque

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

        if len(self.drive.output) != 26 or len(self.drive.input) != 14:
            raise RuntimeError("Expected 26-byte RxPDO and 14-byte TxPDO")

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
            raise RuntimeError(f"Drive did not reach OP: state={self.drive.state}")
        log(
            f"Ready: P={P_GAIN}, I={I_GAIN}, D={D_GAIN}, "
            f"position scale={self.position_scale}:1"
        )

    def _configure_jpvt(self):
        write_i8(self.drive, 0x6060, 0, JPVT_MODE)
        time.sleep(0.1)
        if read_i8(self.drive, 0x6061, 0) != JPVT_MODE:
            raise RuntimeError("JPVT mode was not accepted")

        write_u32(self.drive, 0x34C6, 1, P_GAIN)
        write_u32(self.drive, 0x34C6, 2, I_GAIN)
        write_u32(self.drive, 0x34C6, 3, D_GAIN)
        write_i32(self.drive, 0x34C3, 0, 0)
        write_i32(self.drive, 0x60FF, 0, 0)

        self.position_scale = self._read_position_scale()
        velocity_unit = read_u32(self.drive, 0x60A9, 0)
        if velocity_unit != 0xFDB44700:
            raise RuntimeError(f"Expected milli-rpm velocity units: {velocity_unit:#010x}")

        ssi_config = read_u32(self.drive, 0x3012, 2)
        single_turn_bits = (ssi_config >> 8) & 0xFF
        if single_turn_bits != 12:
            log(
                f"WARNING: assuming HW8A04 resolution (12 bits), but "
                f"0x3012:02 reports {single_turn_bits} bits"
            )
        current_position = read_i32(self.drive, 0x34C6, 6)
        self.target_position = round(current_position / self.position_scale)
        write_i32(self.drive, 0x607A, 0, self.target_position)

        cutoff_hz = round(1 / (2 * CYCLE_S))
        self.drive.sdo_write(0x3676, 1, struct.pack("<H", cutoff_hz))
        self.drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        self.drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))

    def _configure_pdos(self):
        # RxPDO: controlword, velocity, position, torque, P, I, D.
        rx_mappings = (
            0x60400010,
            0x60FF0020,
            0x607A0020,
            0x34C30020,
            0x34C60120,
            0x34C60220,
            0x34C60320,
        )
        self.drive.sdo_write(0x1C12, 0, b"\x00")
        self.drive.sdo_write(0x1603, 0, b"\x00")
        for subindex, mapping in enumerate(rx_mappings, start=1):
            self.drive.sdo_write(0x1603, subindex, struct.pack("<I", mapping))
        self.drive.sdo_write(0x1603, 0, struct.pack("<B", len(rx_mappings)))
        self.drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603))
        self.drive.sdo_write(0x1C12, 0, b"\x01")

        # TxPDO: statusword, filtered velocity/position, filtered joint torque.
        tx_mappings = (0x60410010, 0x34C60B20, 0x34C60A20, 0x36770020)
        self.drive.sdo_write(0x1C13, 0, b"\x00")
        self.drive.sdo_write(0x1A03, 0, b"\x00")
        for subindex, mapping in enumerate(tx_mappings, start=1):
            self.drive.sdo_write(0x1A03, subindex, struct.pack("<I", mapping))
        self.drive.sdo_write(0x1A03, 0, struct.pack("<B", len(tx_mappings)))
        self.drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))
        self.drive.sdo_write(0x1C13, 0, b"\x01")

    def cycle(self, controlword):
        self.drive.output = struct.pack(
            "<HiiiIII",
            controlword,
            self.target_velocity,
            self.target_position,
            self.target_torque,
            self.p_gain,
            self.i_gain,
            self.d_gain,
        )
        self.master.send_processdata()
        wkc = self.master.receive_processdata()
        if wkc <= 0:
            raise RuntimeError(f"EtherCAT receive failed: WKC={wkc}")
        if len(self.drive.input) != 14:
            raise RuntimeError(
                f"Expected 14 input bytes, received {len(self.drive.input)}"
            )
        self.feedback = struct.unpack("<Hiii", self.drive.input)
        return self.feedback

    def run_cycle(self):
        controlword = 0x000F if self.state() == "operation_enabled" else 0x0000
        self.cycle(controlword)

    def _set_controlword(self, controlword, expected_state):
        deadline = time.monotonic() + 1.0
        while True:
            self.cycle(controlword)
            statusword = self.feedback[0]
            if statusword & 0x0008:
                raise RuntimeError(f"Drive fault: SW=0x{statusword:04X}")
            if (statusword & 0x006F) == expected_state:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Controlword 0x{controlword:04X} timed out; "
                    f"SW=0x{statusword:04X}"
                )
            time.sleep(CYCLE_S)

    def enable(self):
        self.target_position = round(self.feedback[2] / self.position_scale)
        for controlword, expected_state in (
            (0x0006, 0x0021),
            (0x0007, 0x0023),
            (0x000F, 0x0027),
        ):
            self._set_controlword(controlword, expected_state)

    def disable(self):
        self._set_controlword(0x0006, 0x0021)
        self._set_controlword(0x0000, 0x0040)

    def move_relative(self, increments):
        if self.state() != "operation_enabled":
            raise ValueError("Drive must be enabled before moving")
        if isinstance(increments, bool) or not isinstance(increments, int):
            raise ValueError("increments must be a whole number")
        current_position = round(self.feedback[2] / self.position_scale)
        target = current_position + increments
        if not -(1 << 31) <= target < (1 << 31):
            raise ValueError("Target exceeds the signed 32-bit range")
        self.target_position = target

    def set_velocity(self, velocity):
        if isinstance(velocity, bool) or not isinstance(velocity, int):
            raise ValueError("velocity must be a whole number")
        if not -(1 << 31) <= velocity < (1 << 31):
            raise ValueError("velocity exceeds the signed 32-bit range")
        self.target_velocity = velocity

    @staticmethod
    def rad_to_inc(position_rad):
        return round(position_rad * POSITION_INCREMENTS_PER_REV / (2 * math.pi))

    @staticmethod
    def inc_to_rad(position_inc):
        return position_inc * (2 * math.pi) / POSITION_INCREMENTS_PER_REV

    @staticmethod
    def rad_s_to_mrpm(velocity_rad_s):
        return round(velocity_rad_s * 60_000 / (2 * math.pi))

    @staticmethod
    def mrpm_to_rad_s(velocity_mrpm):
        return velocity_mrpm * (2 * math.pi) / 60_000

    def capture_origin(self, unitree_position_rad):
        self.epos_origin_inc = self.feedback[2] / self.position_scale
        self.unitree_origin_rad = unitree_position_rad
        self.target_position = round(self.epos_origin_inc)

    def apply_unitree_command(self, position_rad, velocity_rad_s, kp, kd, torque):
        position_delta = position_rad - self.unitree_origin_rad
        self.target_position = round(
            self.epos_origin_inc + self.rad_to_inc(position_delta)
        )
        self.target_velocity = self.rad_s_to_mrpm(velocity_rad_s)
        self.p_gain = round(kp * 1000)       # Nm/rad -> mNm/rad
        self.d_gain = round(kd * 1000)       # Nm*s/rad -> mNm*s/rad
        self.target_torque = round(torque * 1000)  # Nm -> mNm

        if not -(1 << 31) <= self.target_position < (1 << 31):
            raise ValueError("Position command exceeds INTEGER32")
        if not -(1 << 31) <= self.target_velocity < (1 << 31):
            raise ValueError("Velocity command exceeds INTEGER32")
        if not -(1 << 31) <= self.target_torque < (1 << 31):
            raise ValueError("Torque command exceeds INTEGER32")
        for name, value in (("kp", self.p_gain), ("kd", self.d_gain)):
            if not 0 <= value < (1 << 32):
                raise ValueError(f"{name} command exceeds UNSIGNED32")

    def unitree_state(self):
        _, velocity_mrpm, position_milli_inc, torque_mnm = self.feedback
        position_inc = position_milli_inc / self.position_scale
        if hasattr(self, "epos_origin_inc"):
            position_rad = self.unitree_origin_rad + self.inc_to_rad(
                position_inc - self.epos_origin_inc
            )
        else:
            position_rad = self.inc_to_rad(position_inc)
        return position_rad, self.mrpm_to_rad_s(velocity_mrpm), torque_mnm / 1000

    def state(self):
        statusword = self.feedback[0]
        state = statusword & 0x006F
        return {
            0x0040: "switch_on_disabled",
            0x0021: "ready_to_switch_on",
            0x0023: "switched_on",
            0x0027: "operation_enabled",
        }.get(state, "fault" if statusword & 0x0008 else "unknown")

    def status(self):
        statusword, velocity, position = self.feedback
        return {
            "type": "status",
            "state": self.state(),
            "statusword": f"0x{statusword:04X}",
            "position_inc": position / self.position_scale,
            "target_inc": self.target_position,
            "target_velocity": self.target_velocity,
            "velocity_raw": velocity,
            "fault": bool(statusword & 0x0008),
        }

    def _read_position_scale(self):
        target_unit = read_u32(self.drive, 0x60A8, 0)
        fusion_unit = read_u32(self.drive, 0x34C6, 0x0D)
        if target_unit != 0x00B50000:
            raise RuntimeError(
                f"Unsupported target position unit: {target_unit:#010x}"
            )
        if fusion_unit == 0x00B50000:
            return 1
        if fusion_unit == 0xFDB50000:
            return 1000
        raise RuntimeError(f"Unsupported feedback position unit: {fusion_unit:#010x}")

    def _reset_fault_if_needed(self):
        statusword = read_u16(self.drive, 0x6041, 0)
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
        if not statusword & 0x0008:
            return
        log(f"Resetting startup fault: SW=0x{statusword:04X}")
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))
        time.sleep(0.2)
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
        time.sleep(0.2)
        if read_u16(self.drive, 0x6041, 0) & 0x0008:
            raise RuntimeError("Startup fault did not clear")

    def close(self):
        log("Shutting down")
        if self.drive is not None and len(self.drive.output) == 26:
            try:
                for _ in range(100):
                    self.cycle(0x0006)
                    time.sleep(CYCLE_S)
                for _ in range(100):
                    self.cycle(0x0000)
                    time.sleep(CYCLE_S)
            except Exception as ex:
                log(f"Shutdown warning: {ex}")
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
        except Exception:
            pass
        self.master.close()



def write_i8(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<b", value))


def read_i8(drive, index, subindex):
    return struct.unpack("<b", drive.sdo_read(index, subindex)[:1])[0]


def write_i32(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<i", value))


def read_i32(drive, index, subindex):
    return struct.unpack("<i", drive.sdo_read(index, subindex)[:4])[0]


def write_u32(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<I", value))


def read_u16(drive, index, subindex):
    return struct.unpack("<H", drive.sdo_read(index, subindex)[:2])[0]


def read_u32(drive, index, subindex):
    return struct.unpack("<I", drive.sdo_read(index, subindex)[:4])[0]
