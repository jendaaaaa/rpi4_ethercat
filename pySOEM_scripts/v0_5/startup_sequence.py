import json
import math
import os
import socket
import struct
import sys
import time

import pysoem

INTERFACE = "eth0"
SOCKET_PATH = "/tmp/jpvt.sock"

CYCLE_S = 0.002
P_GAIN = 50_000
I_GAIN = 0
D_GAIN = 10_000
JOINT_TORQUE_LIMIT_MNM = 1000
JPVT_MODE = -64

def log(message):
    print(message, file=sys.stderr, flush=True)

class JPVTController:
    
    def __init__(self, interface: str = "eth0"):
        self.master = pysoem.Master()
        self.master.open(interface)
        self.drive = None
        self.position_scale = 1
        self.target_position = 0
        self.target_velocity = 0
        self.target_torque = 0
        self.joint_torque_limit = JOINT_TORQUE_LIMIT_MNM
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.feedback = (0, 0, 0, 0)

    def configure(self):
        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found!")
        
        self.drive = self.master.slaves[0]
        log(f"Found slave: {self.drive.name}")

        # ---------------------------------------------------------------
        # PREOP STATE (DRIVER)
        self.master.state = pysoem.PREOP_STATE
        self.master.write_state()
        if self.drive.state_check(pysoem.PREOP_STATE, 50_000) != pysoem.PREOP_STATE:
            raise RuntimeError("Drive did not reach PREOP")
        log(f"[DRIVER]: PREOP STATE")
        # ready to change parameters

        # ---------------------------------------------------------------
        # RESET FAULT
        statusword = read_u16(self.drive, 0x6041, 0)
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
        if statusword & 0x0008:
            log(f"Resetting startup fault: SW=0x{statusword:04X}")
            self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))
            time.sleep(0.2)
            self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
            time.sleep(0.2)
            if read_u16(self.drive, 0x6041, 0) & 0x0008:
                raise RuntimeError("Startup fault did not clear")
        else:
            log(f"No initial error to clear.")
        
        # ---------------------------------------------------------------
        # CONFIG JPVT
        write_i8(self.drive, 0x6060, 0, JPVT_MODE)
        time.sleep(0.1)
        if read_i8(self.drive, 0x6061, 0) != JPVT_MODE:
            raise RuntimeError("JPVT mode was not accepted")

        write_u32(self.drive, 0x34C6, 1, self.kp)
        write_u32(self.drive, 0x34C6, 2, self.ki)
        write_u32(self.drive, 0x34C6, 3, self.kd)
        write_u32(self.drive, 0x34C6, 4, self.joint_torque_limit)
        write_i32(self.drive, 0x607A, 0, self.target_position)
        write_i32(self.drive, 0x60FF, 0, self.target_velocity)
        write_i32(self.drive, 0x34C3, 0, self.target_torque)
        
        # actual_limit = read_u32(self.drive, 0x34C6, 4)
        # if actual_limit != JOINT_TORQUE_LIMIT_MNM:
        #     raise RuntimeError(f"Torque limit not accepted: {actual_limit}")

        # log(f"Joint torque limit: {actual_limit} mNm")
        
        
        
        target_unit = read_u32(self.drive, 0x60A8, 0)
        fusion_unit = read_u32(self.drive, 0x34C6, 0x0D)
        if target_unit != 0x00B50000:
            raise RuntimeError(
                f"Unsupported target position unit: {target_unit:#010x}"
            )
        if fusion_unit == 0x00B50000:
            self.position_scale = 1
        elif fusion_unit == 0xFDB50000:
            self.position_scale = 1000
        else:
            raise RuntimeError(f"Unsupported feedback position unit: {fusion_unit:#010x}")
        log("Position target and feedback unit captured.")

        cutoff_hz = round(1 / (2 * CYCLE_S))
        # antialiasing cutoff frequency Hz
        self.drive.sdo_write(0x3676, 1, struct.pack("<H", cutoff_hz))
        # interpolation period (value, order)
        self.drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        self.drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))
        
        # ---------------------------------------------------------------
        # CONFIG PDOs
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
        
        # num of assigned RxPDOs set to 0 (turn off)
        self.drive.sdo_write(0x1C12, 0, struct.pack("<B", 0))
        # num of mapped objects in RxPDO4 - set to 0
        self.drive.sdo_write(0x1603, 0, struct.pack("<B", 0))
        for subindex, mapping in enumerate(rx_mappings, start=1):
            # mapped object on position {subindex}
            self.drive.sdo_write(0x1603, subindex, struct.pack("<I", mapping))        
        # num of mapped objects in RxPDO4 - set to length
        self.drive.sdo_write(0x1603, 0, struct.pack("<B", len(rx_mappings)))
        # 1st assigned object to RxPDO set to RxPDO4
        self.drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603))
        # num of assigned RxPDOs set to 1 (turn on)
        self.drive.sdo_write(0x1C12, 0, b"\x01")

        # TxPDO: statusword, filtered velocity/position, filtered joint torque.
        tx_mappings = (
            0x60410010,
            0x34C60B20,
            0x34C60A20,
            0x36770020
        )
        # same for Tx
        self.drive.sdo_write(0x1C13, 0, struct.pack("<B", 0))
        self.drive.sdo_write(0x1A03, 0, struct.pack("<B", 0))
        for subindex, mapping in enumerate(tx_mappings, start=1):
            self.drive.sdo_write(0x1A03, subindex, struct.pack("<I", mapping))
        self.drive.sdo_write(0x1A03, 0, struct.pack("<B", len(tx_mappings)))
        self.drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))
        self.drive.sdo_write(0x1C13, 0, struct.pack("<B", 1))
        
        # ---------------------------------------------------------------
        # RUN CONFIG
        self.master.config_map()

        # checking length of PDOs
        if len(self.drive.output) != 26 or len(self.drive.input) != 14:
            raise RuntimeError("Expected 26-byte RxPDO and 14-byte TxPDO")
        
        # ---------------------------------------------------------------
        # SAFEOP STATE (DRIVER)
        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        if self.master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
            raise RuntimeError("Master did not reach SAFEOP")
        
        log(f"[DRIVER]: SAFEOP STATE")

        # ---------------------------------------------------------------
        # OP STATE (DRIVER)
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        
        log("Feeding watchdog...")
        for _ in range(300):
            self.cycle(0x0000)
            time.sleep(CYCLE_S)
            
        if self.master.state_check(pysoem.OP_STATE, 50_000) != pysoem.OP_STATE:
            raise RuntimeError("Master did not reach OP")
        
        log(f"[DRIVER]: OP STATE")


    def current_position(self):
        return round(self.feedback[2] / self.position_scale)
    

    def cycle(self, controlword):
        self.drive.output = struct.pack(
            "<HiiiIII",
            controlword,
            self.target_velocity,
            self.target_position,
            self.target_torque,
            self.kp,
            self.ki,
            self.kd,
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

    def move_relative(self, increments, kp=None, kd=None):
        if self.state() != "operation_enabled":
            raise ValueError("Drive must be enabled before moving")
        if isinstance(increments, bool) or not isinstance(increments, int):
            raise ValueError("increments must be a whole number")
        current_position = round(self.feedback[2] / self.position_scale)
        target = current_position + increments
        if not -(1 << 31) <= target < (1 << 31):
            raise ValueError("Target exceeds the signed 32-bit range")
        self.kp = gain_to_hej(kp, P_GAIN, "kp")
        self.kd = gain_to_hej(kd, D_GAIN, "kd")
        self.target_position = target

    def set_velocity(self, velocity):
        if isinstance(velocity, bool) or not isinstance(velocity, int):
            raise ValueError("velocity must be a whole number")
        if not -(1 << 31) <= velocity < (1 << 31):
            raise ValueError("velocity exceeds the signed 32-bit range")
        self.target_velocity = velocity

    def state(self):
        statusword = self.feedback[0]
        if statusword & 0x0008:
            return "fault"
        state = statusword & 0x006F
        return {
            0x0040: "switch_on_disabled",
            0x0021: "ready_to_switch_on",
            0x0023: "switched_on",
            0x0027: "operation_enabled",
        }.get(state, "unknown")

    def status(self):
        statusword, velocity, position, torque = self.feedback
        return {
            "type": "status",
            "state": self.state(),
            "statusword": f"0x{statusword:04X}",
            "position_inc": position / self.position_scale,
            "target_inc": self.target_position,
            "target_velocity": self.target_velocity,
            "velocity_raw": velocity,
            "torque_mNm": torque,
            "kp_mNm_per_rad": self.kp,
            "kd_mNm_s_per_rad": self.kd,
            "fault": bool(statusword & 0x0008),
        }


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


def handle_command(controller, command):
    name = command.get("command")
    if name == "status":
        return controller.status(), False
    if name == "enable":
        controller.enable()
        return {"type": "result", "command": name, "ok": True}, False
    if name == "move_relative":
        controller.move_relative(
            command.get("increments"), command.get("kp"), command.get("kd")
        )
        return {
            "type": "result", "command": name, "ok": True,
            "target_inc": controller.target_position,
            "kp_mNm_per_rad": controller.kp,
            "kd_mNm_s_per_rad": controller.kd,
        }, False
    if name == "set_velocity":
        controller.set_velocity(command.get("velocity"))
        return {
            "type": "result", "command": name, "ok": True,
            "target_velocity": controller.target_velocity,
        }, False
    if name == "disable":
        controller.disable()
        return {"type": "result", "command": name, "ok": True}, False
    if name == "quit":
        return {"type": "result", "command": name, "ok": True}, True
    if name == "invalid":
        raise ValueError(command.get("error", "invalid JSON"))
    raise ValueError(f"Unknown command: {name!r}")


def open_server():
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen()
    server.setblocking(False)
    log(f"Listening on {SOCKET_PATH}")
    return server


def receive_command(server, client, data):
    if client is None:
        try:
            client, _ = server.accept()
            client.setblocking(False)
        except BlockingIOError:
            return None, bytearray(), None
    try:
        chunk = client.recv(4096)
    except BlockingIOError:
        return client, data, None
    if not chunk:
        client.close()
        return None, bytearray(), None
    data.extend(chunk)
    if b"\n" not in data:
        return client, data, None
    line = bytes(data).split(b"\n", 1)[0]
    try:
        command = json.loads(line)
    except (UnicodeDecodeError, json.JSONDecodeError) as ex:
        command = {"command": "invalid", "error": str(ex)}
    return client, data, command


def reply(client, message):
    data = json.dumps(message, separators=(",", ":")).encode() + b"\n"
    client.setblocking(True)
    client.settimeout(1.0)
    client.sendall(data)
    client.close()


def run_server(controller):
    server = open_server()
    client = None
    client_data = bytearray()
    quitting = False
    try:
        while not quitting:
            cycle_start = time.monotonic()
            controller.run_cycle()
            client, client_data, command = receive_command(
                server, client, client_data
            )
            if command is not None:
                try:
                    response, quitting = handle_command(controller, command)
                except (ValueError, RuntimeError) as ex:
                    response = {
                        "type": "result", "command": command.get("command"),
                        "ok": False, "error": str(ex),
                    }
                reply(client, response)
                client = None
                client_data = bytearray()
            if controller.feedback[0] & 0x0008:
                raise RuntimeError(
                    f"Drive fault: SW=0x{controller.feedback[0]:04X}"
                )
            remaining = CYCLE_S - (time.monotonic() - cycle_start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if client is not None:
            client.close()
        server.close()


def main():
    controller = JPVTController(INTERFACE)
    try:
        controller.configure()
        run_server(controller)
    except KeyboardInterrupt:
        log("Interrupted")
    except Exception as ex:
        log(f"Fatal: {ex}")
    finally:
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)
        controller.close()


def write_i8(drive, index, subindex, value):
    drive.sdo_write(index, subindex, struct.pack("<b", value))


def gain_to_hej(value, default, name):
    """Convert a command gain from Nm units to the HEJ's mNm units."""
    if value is None:
        return default
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a finite, non-negative number")
    converted = round(value * 1000)
    if converted > 0xFFFFFFFF:
        raise ValueError(f"{name} exceeds the unsigned 32-bit range")
    return converted


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


if __name__ == "__main__":
    main()
