"""Small Unix-socket interface for one EPOS4 drive in JPVT mode.

The process listens on /tmp/jpvt.sock. Each connection sends one JSON command
and receives one JSON response.

Commands:
  {"command":"enable"}
  {"command":"move_relative", "increments":300}
  {"command":"set_velocity", "velocity":0}
  {"command":"status"}
  {"command":"disable"}
  {"command":"quit"}
"""

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
    """Poll the socket without delaying the EtherCAT loop."""
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


def drive_state(statusword):
    state = statusword & 0x006F
    return {
        0x0040: "switch_on_disabled",
        0x0021: "ready_to_switch_on",
        0x0023: "switched_on",
        0x0027: "operation_enabled",
    }.get(state, "fault" if statusword & 0x0008 else "unknown")


def status_message(
    statusword, velocity, position_raw, target, target_velocity, scale
):
    return {
        "type": "status",
        "state": drive_state(statusword),
        "statusword": f"0x{statusword:04X}",
        "position_inc": position_raw / scale,
        "target_inc": target,
        "target_velocity": target_velocity,
        "velocity_raw": velocity,
        "fault": bool(statusword & 0x0008),
    }


def cycle(master, drive, controlword, target_position, target_velocity):
    drive.output = struct.pack("<Hii", controlword, target_velocity, target_position)
    master.send_processdata()
    wkc = master.receive_processdata()
    if wkc <= 0:
        raise RuntimeError(f"EtherCAT receive failed: WKC={wkc}")
    if len(drive.input) != 10:
        raise RuntimeError(f"Expected 10 input bytes, received {len(drive.input)}")
    return struct.unpack("<Hii", drive.input)


def set_controlword(
    master, drive, controlword, expected, target_position, target_velocity
):
    """Send one state-machine command until the statusword confirms it."""
    deadline = time.monotonic() + 1.0
    while True:
        statusword, velocity, position = cycle(
            master, drive, controlword, target_position, target_velocity
        )
        if statusword & 0x0008:
            raise RuntimeError(f"Drive fault: SW=0x{statusword:04X}")
        if (statusword & 0x006F) == expected:
            return statusword, velocity, position
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"Controlword 0x{controlword:04X} timed out; "
                f"SW=0x{statusword:04X}"
            )
        time.sleep(CYCLE_S)


def enable_drive(master, drive, target_position, target_velocity):
    feedback = None
    for controlword, expected in (
        (0x0006, 0x0021),
        (0x0007, 0x0023),
        (0x000F, 0x0027),
    ):
        feedback = set_controlword(master, drive, controlword, expected, target_position, target_velocity)
    return feedback


def disable_drive(master, drive, target_position, target_velocity):
    feedback = set_controlword(
        master, drive, 0x0006, 0x0021, target_position, target_velocity
    )
    return set_controlword(
        master, drive, 0x0000, 0x0040, target_position, target_velocity
    )


def configure_drive(master):
    if master.config_init() <= 0:
        raise RuntimeError("No EtherCAT slaves found")
    drive = master.slaves[0]
    log(f"Found slave: {drive.name}")

    master.state = pysoem.PREOP_STATE
    master.write_state()
    if drive.state_check(pysoem.PREOP_STATE, 50_000) != pysoem.PREOP_STATE:
        raise RuntimeError("Drive did not reach PREOP")

    reset_fault_if_needed(drive)
    write_i8(drive, 0x6060, 0, JPVT_MODE)
    time.sleep(0.1)
    if read_i8(drive, 0x6061, 0) != JPVT_MODE:
        raise RuntimeError("JPVT mode was not accepted")

    write_u32(drive, 0x34C6, 1, P_GAIN)
    write_u32(drive, 0x34C6, 2, I_GAIN)
    write_u32(drive, 0x34C6, 3, D_GAIN)
    write_i32(drive, 0x34C3, 0, 0) # Clear feed-forward target joint torque
    write_i32(drive, 0x60FF, 0, 0) # Clear target velocity

    # Set target position as current position
    position_scale = read_position_scale(drive)
    target_position = round(read_i32(drive, 0x34C6, 6) / position_scale)
    write_i32(drive, 0x607A, 0, target_position)

    # Anti-alias cutoff = half the configured EtherCAT cycle frequency.
    cutoff_hz = round(1 / (2 * CYCLE_S))
    drive.sdo_write(0x3676, 1, struct.pack("<H", cutoff_hz))
    
    # 2 ms interpolation period
    drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
    drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))

    # Assign PDOs to output Sync Managers
    drive.sdo_write(0x1C12, 0, struct.pack("<B", 0))
    drive.sdo_write(0x1C13, 0, struct.pack("<B", 0))

    # RxPDO: controlword, target velocity, target position
    drive.sdo_write(0x1603, 0, struct.pack("<B", 0))
    drive.sdo_write(0x1603, 1, struct.pack("<I", 0x60400010))
    drive.sdo_write(0x1603, 2, struct.pack("<I", 0x60FF0020))
    drive.sdo_write(0x1603, 3, struct.pack("<I", 0x607A0020))
    drive.sdo_write(0x1603, 0, struct.pack("<B", 3))

    # TxPDO: Statusword, filtered JPVT velocity, filtered position
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

    # # RxPDO: controlword, target velocity, target position.
    # drive.sdo_write(0x1C12, 0, b"\x00") # assign to output Sync Master
    # drive.sdo_write(0x1603, 0, b"\x00") # set this PDO to 0 entries
    # for subindex, mapping in enumerate((0x60400010, 0x60FF0020, 0x607A0020), start=1):
    #     drive.sdo_write(0x1603, subindex, struct.pack("<I", mapping))
    # drive.sdo_write(0x1603, 0, b"\x03") # set this PDO to 3 entries
    # drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603)) # selects mapping
    # drive.sdo_write(0x1C12, 0, b"\x01") # activate

    # # TxPDO: statusword, filtered velocity, filtered position.
    # drive.sdo_write(0x1C13, 0, b"\x00")
    # drive.sdo_write(0x1A03, 0, b"\x00")
    # for subindex, mapping in enumerate(
    #     (0x60410010, 0x34C60B20, 0x34C60A20), start=1
    # ):
    #     drive.sdo_write(0x1A03, subindex, struct.pack("<I", mapping))
    # drive.sdo_write(0x1A03, 0, b"\x03")
    # drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))
    # drive.sdo_write(0x1C13, 0, b"\x01")

    master.config_map()
    if len(drive.output) != 10 or len(drive.input) != 10:
        raise RuntimeError("Expected 10-byte RxPDO and TxPDO")

    master.state = pysoem.SAFEOP_STATE
    master.write_state()
    if master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
        raise RuntimeError("Master did not reach SAFEOP")

    master.state = pysoem.OP_STATE
    master.write_state()
    feedback = (0, 0, 0)
    for _ in range(300):
        feedback = cycle(master, drive, 0x0000, target_position, 0)
        time.sleep(CYCLE_S)
    master.read_state()
    if drive.state != pysoem.OP_STATE:
        raise RuntimeError(f"Drive did not reach OP: state={drive.state}")

    log(
        f"Ready: P={P_GAIN}, I={I_GAIN}, D={D_GAIN}, "
        f"position scale={position_scale}:1"
    )
    return drive, position_scale, target_position, feedback


def handle_command(command, master, drive, feedback, target, target_velocity, scale):
    name = command.get("command")

    if name == "status":
        response = status_message(*feedback, target, target_velocity, scale)
        return feedback, target, target_velocity, False, response

    if name == "enable":
        # Hold the measured position before enabling the controller.
        target = round(feedback[2] / scale)
        feedback = enable_drive(master, drive, target, target_velocity)
        response = {"type": "result", "command": name, "ok": True}
        return feedback, target, target_velocity, False, response

    if name == "move_relative":
        if drive_state(feedback[0]) != "operation_enabled":
            raise ValueError("Drive must be enabled before moving")
        increment = command.get("increments")
        if isinstance(increment, bool) or not isinstance(increment, int):
            raise ValueError("increments must be a whole number")
        current_position = round(feedback[2] / scale)
        target = current_position + increment
        if not -(1 << 31) <= target < (1 << 31):
            raise ValueError("Target exceeds the signed 32-bit range")
        response = {
            "type": "result",
            "command": name,
            "ok": True,
            "target_inc": target,
        }
        return feedback, target, target_velocity, False, response

    if name == "set_velocity":
        velocity = command.get("velocity")
        if isinstance(velocity, bool) or not isinstance(velocity, int):
            raise ValueError("velocity must be a whole number")
        if not -(1 << 31) <= velocity < (1 << 31):
            raise ValueError("velocity exceeds the signed 32-bit range")
        target_velocity = velocity
        response = {
            "type": "result",
            "command": name,
            "ok": True,
            "target_velocity": target_velocity,
        }
        return feedback, target, target_velocity, False, response

    if name == "disable":
        feedback = disable_drive(master, drive, target, target_velocity)
        response = {"type": "result", "command": name, "ok": True}
        return feedback, target, target_velocity, False, response

    if name == "quit":
        response = {"type": "result", "command": name, "ok": True}
        return feedback, target, target_velocity, True, response

    if name == "invalid":
        raise ValueError(command.get("error", "invalid JSON"))
    raise ValueError(f"Unknown command: {name!r}")


def main():
    master = pysoem.Master()
    master.open(INTERFACE)
    drive = None
    server = None
    client = None
    client_data = bytearray()
    target_position = 0
    target_velocity = 0

    try:
        drive, scale, target_position, feedback = configure_drive(master)
        server = open_server()
        quitting = False

        while not quitting:
            cycle_start = time.monotonic()

            controlword = (
                0x000F
                if drive_state(feedback[0]) == "operation_enabled"
                else 0x0000
            )
            feedback = cycle(
                master, drive, controlword, target_position, target_velocity
            )

            client, client_data, command = receive_command(
                server, client, client_data
            )

            if command is not None:
                try:
                    (
                        feedback,
                        target_position,
                        target_velocity,
                        quitting,
                        response,
                    ) = handle_command(
                        command, master, drive, feedback,
                        target_position, target_velocity, scale
                    )
                except (ValueError, RuntimeError) as ex:
                    response = {
                        "type": "result",
                        "command": command.get("command"),
                        "ok": False,
                        "error": str(ex),
                    }
                reply(client, response)
                client = None
                client_data = bytearray()

            if feedback[0] & 0x0008:
                raise RuntimeError(f"Drive fault: SW=0x{feedback[0]:04X}")

            remaining = CYCLE_S - (time.monotonic() - cycle_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        log("Interrupted")
    except Exception as ex:
        log(f"Fatal: {ex}")
    finally:
        if client is not None:
            client.close()
        if server is not None:
            server.close()
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)
        shutdown(master, drive, target_position, target_velocity)


def shutdown(master, drive, target_position, target_velocity):
    log("Shutting down")
    if drive is not None and len(drive.output) == 10:
        try:
            for _ in range(100):
                cycle(master, drive, 0x0006, target_position, target_velocity)
                time.sleep(CYCLE_S)
            for _ in range(100):
                cycle(master, drive, 0x0000, target_position, target_velocity)
                time.sleep(CYCLE_S)
        except Exception as ex:
            log(f"Shutdown warning: {ex}")
    try:
        master.state = pysoem.INIT_STATE
        master.write_state()
    except Exception:
        pass
    master.close()


def reset_fault_if_needed(drive):
    statusword = read_u16(drive, 0x6041, 0)
    drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
    if not statusword & 0x0008:
        return
    log(f"Resetting startup fault: SW=0x{statusword:04X}")
    drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))
    time.sleep(0.2)
    drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
    time.sleep(0.2)
    if read_u16(drive, 0x6041, 0) & 0x0008:
        raise RuntimeError("Startup fault did not clear")


def read_position_scale(drive):
    target_unit = read_u32(drive, 0x60A8, 0)
    fusion_unit = read_u32(drive, 0x34C6, 0x0D)
    if target_unit != 0x00B50000:
        raise RuntimeError(f"Unsupported target position unit: {target_unit:#010x}")
    if fusion_unit == 0x00B50000:
        return 1
    if fusion_unit == 0xFDB50000:
        return 1000
    raise RuntimeError(f"Unsupported feedback position unit: {fusion_unit:#010x}")


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
