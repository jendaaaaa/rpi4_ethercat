import json
import math
import os
import socket
import struct
import sys
import time

from enum import IntEnum, Enum
from dataclasses import dataclass

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

@dataclass(frozen=True)
class SDOParameter:
    index:      int
    subindex:   int
    fmt:        str
    unit:       str = ""
    # minimum:    float | None = None
    # maximum:    float | None = None
    # writable:   bool = True

class SDO(Enum):
    CONTROLWORD             = SDOParameter(0x6040, 0x00, "<H")
    STATUSWORD              = SDOParameter(0x6041, 0x00, "<H")

    OPERATION_MODE          = SDOParameter(0x6060, 0x00, "<b")
    OPERATION_MODE_DISPLAY  = SDOParameter(0x6061, 0x00, "<b")

    P_GAIN                  = SDOParameter(0x34C6, 0x01, "<I", "mNm/rad")
    I_GAIN                  = SDOParameter(0x34C6, 0x02, "<I", "mNm/(rad*s)")
    D_GAIN                  = SDOParameter(0x34C6, 0x03, "<I", "mNm*s/rad")
    JOINT_TORQUE_LIMIT      = SDOParameter(0x34C6, 0x04, "<I", "mNm")

    TARGET_POSITION         = SDOParameter(0x607A, 0x00, "<i", "inc")
    TARGET_VELOCITY         = SDOParameter(0x60FF, 0x00, "<i", "mrpm")
    TARGET_JOINT_TORQUE     = SDOParameter(0x34C3, 0x00, "<i", "mNm")

    TARGET_POSITION_UNIT    = SDOParameter(0x60A8, 0x00, "<I")
    FUSION_POSITION_UNIT    = SDOParameter(0x34C6, 0x0D, "<I")

    ANTI_ALIAS_CUTOFF       = SDOParameter(0x3676, 0x01, "<H", "Hz")
    INTERPOLATION_PERIOD    = SDOParameter(0x60C2, 0x01, "<B")
    INTERPOLATION_EXPONENT  = SDOParameter(0x60C2, 0x02, "<b")
    
    N_MAPPED_RXPDO          = SDOParameter(0x1C12, 0x00, "<B")
    N_MAPPED_TXPDO          = SDOParameter(0x1C13, 0x00, "<B")
    
    OBJ_1_RXPDO             = SDOParameter(0x1C12, 0x01, "<H")
    OBJ_1_TXPDO             = SDOParameter(0x1C13, 0x01, "<H")
    
    N_MAPPED_OBJECTS_RXPDO4 = SDOParameter(0x1603, 0x00, "<B")
    N_MAPPED_OBJECTS_TXPDO4 = SDOParameter(0x1A03, 0x00, "<B")
    
    OBJ_1_RXPDO4            = SDOParameter(0x1603, 0x01, "<B")
    OBJ_2_RXPDO4            = SDOParameter(0x1603, 0x02, "<B")
    OBJ_3_RXPDO4            = SDOParameter(0x1603, 0x03, "<B")
    OBJ_4_RXPDO4            = SDOParameter(0x1603, 0x04, "<B")
    OBJ_5_RXPDO4            = SDOParameter(0x1603, 0x05, "<B")
    OBJ_6_RXPDO4            = SDOParameter(0x1603, 0x06, "<B")
    
    OBJ_1_TXPDO4            = SDOParameter(0x1A03, 0x01, "<B")
    OBJ_2_TXPDO4            = SDOParameter(0x1A03, 0x02, "<B")
    OBJ_3_TXPDO4            = SDOParameter(0x1A03, 0x03, "<B")
    OBJ_4_TXPDO4            = SDOParameter(0x1A03, 0x04, "<B")
    OBJ_5_TXPDO4            = SDOParameter(0x1A03, 0x05, "<B")
    OBJ_6_TXPDO4            = SDOParameter(0x1A03, 0x06, "<B")

class RX_MAPPING(IntEnum):
    CONTROLWORD         = 0x60400010
    VELOCITY            = 0x60FF0020
    POSITION            = 0x607A0020
    TORQUE              = 0x34C30020
    P_GAIN              = 0x34C60120
    I_GAIN              = 0x34C60220
    D_GAIN              = 0x34C60320
    
class TX_MAPPING(IntEnum):
    STATUSWORD              = 0x60410010
    FILTERED_VELOCITY       = 0x34C60B20
    FILTERED_POSITION       = 0x34C60A20
    FILTERED_JOINT_TORQUE   = 0x36770020

# EPOS State
class STATE(IntEnum):
    MASK_STATE          = 0b01101111
    MASK_FAULT          = 0b00001000
    
    SWITCH_ON_DISABLED  = 0b01000000    # 0x40
    READY_TO_SWITCH_ON  = 0b00100001    # 0x21    
    SWITCHED_ON         = 0b00100011    # 0x23
    OPERATION_ENABLED   = 0b00100111    # 0x27

# ControlWord
class CW(IntEnum):
    SHUTDOWN            = 0b00000110    # 0x0006
    SWITCH_ON           = 0b00000111    # 0x0007
    ENABLE_OPERATION    = 0b00001111    # 0x000F
    DISABLE_VOLTAGE     = 0b00000000    # 0x0000
    FAULT_RESET_0       = 0b00000000    # 0x0000
    FAULT_RESET_1       = 0b10000000    # 0x0080

class JPVTController:
    def __init__(self, interface: str = "eth0"):
        self.master = pysoem.Master()
        self.master.open(interface)
        self.drive = None
        self.position_scale = 1
        self.target_position = 0
        self.target_velocity = 0
        self.target_joint_torque = 0
        self.joint_torque_limit = JOINT_TORQUE_LIMIT_MNM
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.feedback = (0, 0, 0, 0)
    
    def configure(self) -> None:
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
        statusword = self.get_sdo(SDO.STATUSWORD)
        self.set_sdo(SDO.CONTROLWORD, CW.FAULT_RESET_0)
        if statusword & STATE.MASK_FAULT:
            log(f"Resetting startup fault: SW=0x{statusword:04X}")
            self.set_sdo(SDO.CONTROLWORD, CW.FAULT_RESET_1)
            time.sleep(0.2)
            self.set_sdo(SDO.CONTROLWORD, CW.FAULT_RESET_0)
            time.sleep(0.2)
            if self.get_sdo(SDO.CONTROLWORD) & STATE.MASK_FAULT:
                raise RuntimeError("Startup fault did not clear!")
        
        # ---------------------------------------------------------------
        # CONFIG JPVT
        self.set_sdo(SDO.OPERATION_MODE, JPVT_MODE)
        time.sleep(0.1)
        if self.get_sdo(SDO.OPERATION_MODE_DISPLAY) != JPVT_MODE:
            raise RuntimeError("JPVT mode was not accepted!")  

        self.set_sdo(SDO.P_GAIN, self.kp)
        self.set_sdo(SDO.I_GAIN, self.ki)
        self.set_sdo(SDO.D_GAIN, self.kd)
        self.set_sdo(SDO.TARGET_POSITION, self.target_position)
        self.set_sdo(SDO.TARGET_VELOCITY, self.target_velocity)
        self.set_sdo(SDO.TARGET_JOINT_TORQUE, self.target_joint_torque)
        self.set_sdo(SDO.JOINT_TORQUE_LIMIT, self.joint_torque_limit)
        
        actual_limit = self.get_sdo(SDO.JOINT_TORQUE_LIMIT)
        if actual_limit != self.joint_torque_limit:
            raise RuntimeError(f"Torque limit not accepted... Actual limit = {actual_limit} ")
        log(f"Joint torque limit: {actual_limit} mNm")
        
        target_unit = self.get_sdo(SDO.TARGET_POSITION_UNIT)
        fusion_unit = self.get_sdo(SDO.FUSION_POSITION_UNIT)
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
        self.set_sdo(SDO.ANTI_ALIAS_CUTOFF, cutoff_hz)
        self.set_sdo(SDO.INTERPOLATION_PERIOD, 2)
        self.set_sdo(SDO.INTERPOLATION_EXPONENT, -3)
        
        # ---------------------------------------------------------------
        # CONFIG PDO RX
        self.set_sdo(SDO.N_MAPPED_RXPDO, 0)
        self.set_sdo(SDO.N_MAPPED_OBJECTS_RXPDO4, 0)
        
        self.set_sdo(SDO.OBJ_1_RXPDO4, RX_MAPPING.CONTROLWORD)
        self.set_sdo(SDO.OBJ_2_RXPDO4, RX_MAPPING.VELOCITY)
        self.set_sdo(SDO.OBJ_3_RXPDO4, RX_MAPPING.POSITION)
        self.set_sdo(SDO.OBJ_4_RXPDO4, RX_MAPPING.P_GAIN)
        self.set_sdo(SDO.OBJ_5_RXPDO4, RX_MAPPING.I_GAIN)
        self.set_sdo(SDO.OBJ_6_RXPDO4, RX_MAPPING.D_GAIN)
        
        self.set_sdo(SDO.N_MAPPED_OBJECTS_RXPDO4, 6)
        
        rxpdo_object_index = SDO.OBJ_1_RXPDO4.value.index
        self.set_sdo(SDO.OBJ_1_RXPDO, rxpdo_object_index)
        self.set_sdo(SDO.N_MAPPED_RXPDO, 1)
        
        # ---------------------------------------------------------------
        # CONFIG PDO TX
        self.set_sdo(SDO.N_MAPPED_TXPDO, 0)
        self.set_sdo(SDO.N_MAPPED_OBJECTS_TXPDO4, 0)
        
        self.set_sdo(SDO.OBJ_1_TXPDO4, TX_MAPPING.STATUSWORD)
        self.set_sdo(SDO.OBJ_2_TXPDO4, TX_MAPPING.FILTERED_VELOCITY)
        self.set_sdo(SDO.OBJ_3_TXPDO4, TX_MAPPING.FILTERED_POSITION)
        self.set_sdo(SDO.OBJ_4_TXPDO4, TX_MAPPING.FILTERED_JOINT_TORQUE)
        
        self.set_sdo(SDO.N_MAPPED_OBJECTS_TXPDO4, 4)
        
        txpdo_object_index = SDO.OBJ_1_TXPDO4.value.index
        self.set_sdo(SDO.OBJ_1_TXPDO, txpdo_object_index)
        self.set_sdo(SDO.N_MAPPED_TXPDO, 1)
        
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
        
        log("Feeding watchdog to set OP_STATE...")
        for _ in range(300):
            self.cycle(CW.DISABLE_VOLTAGE)
            time.sleep(CYCLE_S)
            
        if self.master.state_check(pysoem.OP_STATE, 50_000) != pysoem.OP_STATE:
            raise RuntimeError("Master did not reach OP")
        
        log(f"[DRIVER]: OP STATE")


    def set_sdo(self, parameter: SDO, value):
        p = parameter.value
        self.drive.sdo_write(p.index, p.subindex, struct.pack(p.fmt, value))


    def get_sdo(self, parameter: SDO):
        p = parameter.value
        size = struct.calcsize(p.fmt)
        data = self.drie.sdo_read(p.index, p.subindex)
        return struct.unpack(p.fmt, data[:size])[0]


    def current_position(self) -> float:
        return round(self.feedback[2] / self.position_scale)
    

    def cycle(self, controlword) -> tuple:
        self.drive.output = struct.pack(
            "<HiiiIII",
            controlword,
            self.target_velocity,
            self.target_position,
            self.target_joint_torque,
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


    # def run_cycle(self) -> None:
    #     controlword = ControlWord.ENABLE_OPERATION if self.state() == "operation_enabled" else 0x0000
    #     self.cycle(controlword)

    
    def _set_cw(self, controlword: CW, expected_state: STATE):
        deadline = time.monotonic() + 1.0
        while True:
            self.cycle(controlword)
            statusword = self.feedback[0]
            if statusword & STATE.MASK_FAULT:
                raise RuntimeError(f"Drive fault: SW=0x{statusword:04X}")
            if (statusword & STATE.MASK_STATE) == expected_state:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Controlword 0x{controlword:04X} timed out; "
                    f"SW=0x{statusword:04X}"
                )
            time.sleep(CYCLE_S)
    

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
        self.target_position = self.current_position()
        self._set_cw(CW.SHUTDOWN, STATE.READY_TO_SWITCH_ON)
        self._set_cw(CW.SWITCH_ON, STATE.SWITCHED_ON)
        self._set_cw(CW.ENABLE_OPERATION, STATE.OPERATION_ENABLED)

    def disable(self):
        self._set_cw(CW.SHUTDOWN, STATE.READY_TO_SWITCH_ON)
        self._set_cw(CW.DISABLE_VOLTAGE, STATE.SWITCH_ON_DISABLED)




    def move_relative(self, increments, kp=None, kd=None):
        if self.state() != STATE.OPERATION_ENABLED:
            raise ValueError("Drive must be enabled before moving")
        if isinstance(increments, bool) or not isinstance(increments, int):
            raise ValueError("increments must be a whole number")
        
        current_position = self.current_position()
        target = current_position + increments
        if not -(1 << 31) <= target < (1 << 31):
            raise ValueError("Target exceeds the signed 32-bit range")
        self.kp = gain_to_hej(kp, P_GAIN, "kp")
        self.kd = gain_to_hej(kd, D_GAIN, "kd")
        self.target_position = target

    # def set_velocity(self, velocity):
    #     pass

    def state(self):
        statusword = self.feedback[0]
        if statusword & STATE.MASK_FAULT:
            return STATE.MASK_FAULT
        current_state = statusword & STATE.MASK_STATE
        if current_state not in STATE:
            return None
        return current_state
    
    def state_name(self):
        current_state = self.state()
        if current_state is None:
            return "UNKNOWN"
        return STATE(current_state).name

    def status(self):
        statusword, velocity, position, torque = self.feedback
        return {
            "type": "status",
            "state": self.state_name(),
            "statusword": f"0x{statusword:04X}",
            "position_inc": position / self.position_scale,
            "target_inc": self.target_position,
            "target_velocity": self.target_velocity,
            "velocity_raw": velocity,
            "torque_mNm": torque,
            "kp_mNm_per_rad": self.kp,
            "kd_mNm_s_per_rad": self.kd,
            "fault": bool(statusword & STATE.MASK_FAULT),
        }


    def close(self):
        log("Shutting down")
        if self.drive is not None and len(self.drive.output) == 26:
            try:
                for _ in range(100):
                    self.cycle(CW.SHUTDOWN)
                    time.sleep(CYCLE_S)
                for _ in range(100):
                    self.cycle(CW.DISABLE_VOLTAGE)
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
    # if name == "set_velocity":
    #     controller.set_velocity(command.get("velocity"))
    #     return {
    #         "type": "result", "command": name, "ok": True,
    #         "target_velocity": controller.target_velocity,
    #     }, False
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
            
            state = controller.state()
            if state == STATE.OPERATION_ENABLED:
                cw = CW.ENABLE_OPERATION
            else:
                cw = CW.DISABLE_VOLTAGE
            
            controller.cycle(cw)
            
            client, client_data, command = receive_command(server, client, client_data)
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
            if controller.state() == "fault":
                raise RuntimeError(f"Drive fault: SW=0x{controller.feedback[0]:04X}")
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
