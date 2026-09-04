"""Unix-socket controller for one EPOS4 drive in JPVT mode."""

import json
import os
import socket
import struct
import sys
import time

import pysoem

INTERFACE = "eth0"
SOCKET_PATH = "/tmp/jpvt.sock"
CYCLE_S = 0.002
JPVT_MODE = -64
P_GAIN = 50_000
I_GAIN = 0
D_GAIN = 10_000


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
        self.feedback = (0, 0, 0)

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

        if len(self.drive.output) != 10 or len(self.drive.input) != 10:
            raise RuntimeError("Expected 10-byte RxPDO and TxPDO")

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
        current_position = read_i32(self.drive, 0x34C6, 6)
        self.target_position = round(current_position / self.position_scale)
        write_i32(self.drive, 0x607A, 0, self.target_position)

        cutoff_hz = round(1 / (2 * CYCLE_S))
        self.drive.sdo_write(0x3676, 1, struct.pack("<H", cutoff_hz))
        self.drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        self.drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))

    def _configure_pdos(self):
        # RxPDO: controlword, target velocity, target position.
        self.drive.sdo_write(0x1C12, 0, b"\x00")
        self.drive.sdo_write(0x1603, 0, b"\x00")
        for subindex, mapping in enumerate(
            (0x60400010, 0x60FF0020, 0x607A0020), start=1
        ):
            self.drive.sdo_write(0x1603, subindex, struct.pack("<I", mapping))
        self.drive.sdo_write(0x1603, 0, b"\x03")
        self.drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603))
        self.drive.sdo_write(0x1C12, 0, b"\x01")

        # TxPDO: statusword, filtered velocity, filtered position.
        self.drive.sdo_write(0x1C13, 0, b"\x00")
        self.drive.sdo_write(0x1A03, 0, b"\x00")
        for subindex, mapping in enumerate(
            (0x60410010, 0x34C60B20, 0x34C60A20), start=1
        ):
            self.drive.sdo_write(0x1A03, subindex, struct.pack("<I", mapping))
        self.drive.sdo_write(0x1A03, 0, b"\x03")
        self.drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))
        self.drive.sdo_write(0x1C13, 0, b"\x01")

    def cycle(self, controlword):
        self.drive.output = struct.pack(
            "<Hii", controlword, self.target_velocity, self.target_position
        )
        self.master.send_processdata()
        wkc = self.master.receive_processdata()
        if wkc <= 0:
            raise RuntimeError(f"EtherCAT receive failed: WKC={wkc}")
        if len(self.drive.input) != 10:
            raise RuntimeError(
                f"Expected 10 input bytes, received {len(self.drive.input)}"
            )
        self.feedback = struct.unpack("<Hii", self.drive.input)
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
        if self.drive is not None and len(self.drive.output) == 10:
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
        controller.move_relative(command.get("increments"))
        return {
            "type": "result", "command": name, "ok": True,
            "target_inc": controller.target_position,
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
