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

N_CYCLES_SENSING_POSITION = 1_000
N_CYCLES_ZERO_POSITION = 500

P_GAIN_MOVE_TARGET = 40_000
I_GAIN_MOVE_TARGET = 0
D_GAIN_MOVE_TARGET = 1_000

P_GAIN_MOVE_TO_ZERO = 38_000
I_GAIN_MOVE_TO_ZERO = 0
D_GAIN_MOVE_TO_ZERO = 1_100

P_GAIN_HOLD = 60_000
I_GAIN_HOLD = 0
D_GAIN_HOLD = 2_000

P_GAIN_DAMPING = 0
I_GAIN_DAMPING = 0
D_GAIN_DAMPING = 2_000

ZERO_MARGIN = 10

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
    
    OBJ_1_RXPDO4            = SDOParameter(0x1603, 0x01, "<I")
    OBJ_2_RXPDO4            = SDOParameter(0x1603, 0x02, "<I")
    OBJ_3_RXPDO4            = SDOParameter(0x1603, 0x03, "<I")
    OBJ_4_RXPDO4            = SDOParameter(0x1603, 0x04, "<I")
    OBJ_5_RXPDO4            = SDOParameter(0x1603, 0x05, "<I")
    OBJ_6_RXPDO4            = SDOParameter(0x1603, 0x06, "<I")
    OBJ_7_RXPDO4            = SDOParameter(0x1603, 0x07, "<I")
    
    OBJ_1_TXPDO4            = SDOParameter(0x1A03, 0x01, "<I")
    OBJ_2_TXPDO4            = SDOParameter(0x1A03, 0x02, "<I")
    OBJ_3_TXPDO4            = SDOParameter(0x1A03, 0x03, "<I")
    OBJ_4_TXPDO4            = SDOParameter(0x1A03, 0x04, "<I")
    OBJ_5_TXPDO4            = SDOParameter(0x1A03, 0x05, "<I")
    OBJ_6_TXPDO4            = SDOParameter(0x1A03, 0x06, "<I")

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
class EPOS_STATE(IntEnum):
    MASK_STATE          = 0b01101111
    MASK_FAULT          = 0b00001000
    
    SWITCH_ON_DISABLED  = 0b01000000    # 0x40
    READY_TO_SWITCH_ON  = 0b00100001    # 0x21    
    SWITCHED_ON         = 0b00100011    # 0x23
    OPERATION_ENABLED   = 0b00100111    # 0x27
    
class MAIN_STATE(IntEnum):
    FAULT               = -1
    DEAD                = 0
    POWERED             = 1
    INITIALIZED         = 2
    SENSING_POSITION    = 3
    POSITION_READY      = 4
    MOVING_TO_ZERO      = 5
    ZERO_READY          = 6
    DAMPING             = 7

# ControlWord
class CW(IntEnum):
    SHUTDOWN            = 0b00000110    # 0x0006
    SWITCH_ON           = 0b00000111    # 0x0007
    ENABLE_OPERATION    = 0b00001111    # 0x000F
    DISABLE_VOLTAGE     = 0b00000000    # 0x0000
    FAULT_RESET_0       = 0b00000000    # 0x0000
    FAULT_RESET_1       = 0b10000000    # 0x0080
    
ALLOWED_TRANSITIONS = {
    MAIN_STATE.DEAD: {
        MAIN_STATE.POWERED
    },
    MAIN_STATE.POWERED: {
        MAIN_STATE.INITIALIZED
    },
    MAIN_STATE.INITIALIZED: {
        MAIN_STATE.SENSING_POSITION
    },
    MAIN_STATE.SENSING_POSITION: {
        MAIN_STATE.INITIALIZED,
        MAIN_STATE.POSITION_READY
    },
    MAIN_STATE.POSITION_READY: {
        MAIN_STATE.INITIALIZED,
        MAIN_STATE.MOVING_TO_ZERO
    },
    MAIN_STATE.MOVING_TO_ZERO: {
        MAIN_STATE.INITIALIZED,
        MAIN_STATE.ZERO_READY
    },
    MAIN_STATE.ZERO_READY: {
        MAIN_STATE.INITIALIZED,
        MAIN_STATE.DAMPING  
    },
    MAIN_STATE.DAMPING: {
        MAIN_STATE.INITIALIZED,
        MAIN_STATE.MOVING_TO_ZERO
    }
}

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
        self.state = MAIN_STATE.DEAD
        self.state_requested = None
    
    def configure(self) -> None:
        self.state = MAIN_STATE.POWERED
        if self.master.config_init() <= 0:
            raise RuntimeError("[EtherCat] No EtherCAT slaves found!")
        
        self.drive = self.master.slaves[0]
        log(f"[EtherCat] Found slave: {self.drive.name}")

        # ---------------------------------------------------------------
        # PREOP STATE (DRIVER)
        self.master.state = pysoem.PREOP_STATE
        self.master.write_state()
        if self.drive.state_check(pysoem.PREOP_STATE, 50_000) != pysoem.PREOP_STATE:
            raise RuntimeError("[EtherCat] Drive did not reach PREOP")
        log(f"[EtherCat] State: PREOP")
        # ready to change parameters

        # ---------------------------------------------------------------
        # RESET FAULT
        statusword = self.get_sdo(SDO.STATUSWORD)
        self.set_sdo(SDO.CONTROLWORD, CW.FAULT_RESET_0)
        if statusword & EPOS_STATE.MASK_FAULT:
            log(f"[EPOS] Resetting startup fault: SW=0x{statusword:04X}")
            self.set_sdo(SDO.CONTROLWORD, CW.FAULT_RESET_1)
            time.sleep(0.2)
            self.set_sdo(SDO.CONTROLWORD, CW.FAULT_RESET_0)
            time.sleep(0.2)
            if self.get_sdo(SDO.STATUSWORD) & EPOS_STATE.MASK_FAULT:
                raise RuntimeError("[EPOS] Startup fault did not clear!")
        else:
            log(f"[EPOS] Startup fault clear")
        
        # ---------------------------------------------------------------
        # CONFIG JPVT
        self.set_sdo(SDO.OPERATION_MODE, JPVT_MODE)
        time.sleep(0.1)
        if self.get_sdo(SDO.OPERATION_MODE_DISPLAY) != JPVT_MODE:
            raise RuntimeError("[EPOS] JPVT mode was not accepted!")  

        # ---------------------------------------------------------------
        # CONFIG PARAMS
        self.set_sdo(SDO.P_GAIN, self.kp)
        self.set_sdo(SDO.I_GAIN, self.ki)
        self.set_sdo(SDO.D_GAIN, self.kd)
        self.set_sdo(SDO.TARGET_POSITION, self.target_position)
        self.set_sdo(SDO.TARGET_VELOCITY, self.target_velocity)
        self.set_sdo(SDO.TARGET_JOINT_TORQUE, self.target_joint_torque)
        self.set_sdo(SDO.JOINT_TORQUE_LIMIT, self.joint_torque_limit)
        
        actual_limit = self.get_sdo(SDO.JOINT_TORQUE_LIMIT)
        if actual_limit != self.joint_torque_limit:
            raise RuntimeError(f"[EPOS] Torque limit not accepted... Actual limit = {actual_limit} ")
        log(f"[EPOS] Joint torque limit: {actual_limit} mNm")
    
        # ---------------------------------------------------------------
        # CONFIG UNITS
        target_unit = self.get_sdo(SDO.TARGET_POSITION_UNIT)
        fusion_unit = self.get_sdo(SDO.FUSION_POSITION_UNIT)
        if target_unit != 0x00B50000:
            raise RuntimeError(
                f"[EPOS] Unsupported target position unit: {target_unit:#010x}"
            )
        if fusion_unit == 0x00B50000:
            self.position_scale = 1
        elif fusion_unit == 0xFDB50000:
            self.position_scale = 1000
        else:
            raise RuntimeError(f"[EPOS] Unsupported feedback position unit: {fusion_unit:#010x}")
        log("[EPOS] Position target and feedback unit captured")
        
        # ---------------------------------------------------------------
        # CONFIG FILTERING
        cutoff_hz = round(1 / (2 * CYCLE_S))
        self.set_sdo(SDO.ANTI_ALIAS_CUTOFF, cutoff_hz)
        self.set_sdo(SDO.INTERPOLATION_PERIOD, 2)
        self.set_sdo(SDO.INTERPOLATION_EXPONENT, -3)
        log("[EPOS] Antialiasing and interpolation period set")
        
        # ---------------------------------------------------------------
        # CONFIG PDO RX
        self.set_sdo(SDO.N_MAPPED_RXPDO, 0)
        self.set_sdo(SDO.N_MAPPED_OBJECTS_RXPDO4, 0)
        
        self.set_sdo(SDO.OBJ_1_RXPDO4, RX_MAPPING.CONTROLWORD)
        self.set_sdo(SDO.OBJ_2_RXPDO4, RX_MAPPING.VELOCITY)
        self.set_sdo(SDO.OBJ_3_RXPDO4, RX_MAPPING.POSITION)
        self.set_sdo(SDO.OBJ_4_RXPDO4, RX_MAPPING.TORQUE)
        self.set_sdo(SDO.OBJ_5_RXPDO4, RX_MAPPING.P_GAIN)
        self.set_sdo(SDO.OBJ_6_RXPDO4, RX_MAPPING.I_GAIN)
        self.set_sdo(SDO.OBJ_7_RXPDO4, RX_MAPPING.D_GAIN)
        
        self.set_sdo(SDO.N_MAPPED_OBJECTS_RXPDO4, 7)
        
        rxpdo_object_index = SDO.OBJ_1_RXPDO4.value.index
        self.set_sdo(SDO.OBJ_1_RXPDO, rxpdo_object_index)
        self.set_sdo(SDO.N_MAPPED_RXPDO, 1)
        log("[EPOS] RxPDO set")
        
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
        log("[EPOS] TxPDO set")
        
        # ---------------------------------------------------------------
        # RUN CONFIG
        self.master.config_map()
        log("[EtherCat] Master Config mapped")

        # checking length of PDOs
        output_size = len(self.drive.output)
        input_size = len(self.drive.input)
        if output_size != 26 or input_size != 14:
            raise RuntimeError(f"[EtherCat] Expected 26-byte RxPDO and 14-byte TxPDO. Received RxPDO {output_size} and TxPDO {input_size}!")
        
        # ---------------------------------------------------------------
        # SAFEOP STATE (DRIVER)
        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        if self.master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
            raise RuntimeError("[EtherCat] Master did not reach SAFEOP")
        
        log(f"[EtherCat] State: SAFEOP")

        # ---------------------------------------------------------------
        # OP STATE (DRIVER)
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        
        log("[EtherCat] Feeding watchdog to set OP_STATE...")
        for _ in range(300):
            self.cycle(CW.DISABLE_VOLTAGE)
            time.sleep(CYCLE_S)
            
        if self.master.state_check(pysoem.OP_STATE, 50_000) != pysoem.OP_STATE:
            raise RuntimeError("[EtherCat] Master did not reach OP")
        
        log(f"[EtherCat] State: OP")
        self.state = MAIN_STATE.INITIALIZED

    def set_sdo(self, parameter: SDO, value):
        p = parameter.value
        self.drive.sdo_write(p.index, p.subindex, struct.pack(p.fmt, value))

    def get_sdo(self, parameter: SDO):
        p = parameter.value
        size = struct.calcsize(p.fmt)
        data = self.drive.sdo_read(p.index, p.subindex)
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
            raise RuntimeError(f"[EtherCat] Receive failed: WKC={wkc}")
        if len(self.drive.input) != 14:
            raise RuntimeError(
                f"[EtherCat] Expected 14 input bytes, received {len(self.drive.input)}"
            )
        self.feedback = struct.unpack("<Hiii", self.drive.input)
        return self.feedback
    
    def _set_cw(self, controlword: CW, expected_state: EPOS_STATE):
        deadline = time.monotonic() + 1.0
        while True:
            self.cycle(controlword)
            statusword = self.feedback[0]
            if statusword & EPOS_STATE.MASK_FAULT:
                raise RuntimeError(f"[EPOS] fault: SW=0x{statusword:04X}")
            if (statusword & EPOS_STATE.MASK_STATE) == expected_state:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"[EPOS] Controlword 0x{controlword:04X} timed out; "
                    f"SW=0x{statusword:04X}"
                )
            time.sleep(CYCLE_S)

    def enable(self):
        self.target_position = self.current_position()
        self.target_velocity = 0
        self.target_joint_torque = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self._set_cw(CW.SHUTDOWN, EPOS_STATE.READY_TO_SWITCH_ON)
        self._set_cw(CW.SWITCH_ON, EPOS_STATE.SWITCHED_ON)
        self._set_cw(CW.ENABLE_OPERATION, EPOS_STATE.OPERATION_ENABLED)

    def disable(self):
        self._set_cw(CW.SHUTDOWN, EPOS_STATE.READY_TO_SWITCH_ON)
        self._set_cw(CW.DISABLE_VOLTAGE, EPOS_STATE.SWITCH_ON_DISABLED)
        
    def move_target(self, q, dq = 0, kp = None, kd = None):
        if self.get_epos_state() != EPOS_STATE.OPERATION_ENABLED:
            raise ValueError("[EPOS] Drive must be enabled before moving")
        if self.get_main_state() != MAIN_STATE.DAMPING:
            raise ValueError("[Main] Main must be in DAMPING mode before moving")
        if not (self._valid_target(q, "q") and self._valid_target(dq, "dq")):
            raise ValueError("[Main] Skipped move command")
        
        kp = self._norm_gain(kp, P_GAIN_MOVE_TARGET, "kp")
        kd = self._norm_gain(kd, D_GAIN_MOVE_TARGET, "kd")
        # log(f"q = {q}, dq = {dq}, kp = {kp}, kd = {kd}")
        
        self.kp = kp
        self.kd = kd
        self.target_position = q
        self.target_velocity = dq

    def _valid_target(self, value, name:str = "none") -> bool:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"[Main] Value of {name} must be a whole number")
        if not -(1 << 31) <= value < (1 << 31):
            raise ValueError(f"[Main] Value of {name} exceeds the signed 32-bit range")
        return True

    def _norm_gain(self, value, default, name):
        if value is None:
            return default
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"[Main] {name} must be a number")
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"[Main] {name} must be a finite, non-negative number")
        converted = round(value * 1000)
        if converted > 0xFFFFFFFF:
            raise ValueError(f"[Main] {name} exceeds the unsigned 32-bit range")
        return converted

    def get_epos_state(self):
        statusword = self.feedback[0]
        if statusword & EPOS_STATE.MASK_FAULT:
            return EPOS_STATE.MASK_FAULT
        current_state = statusword & EPOS_STATE.MASK_STATE
        if current_state not in EPOS_STATE:
            return None
        return current_state
    
    def get_epos_state_name(self):
        current_state = self.get_epos_state()
        if current_state is None:
            return "UNKNOWN"
        return EPOS_STATE(current_state).name
    
    def get_main_state(self):
        return self.state
    
    def get_main_state_name(self):
        return MAIN_STATE(self.state).name
    
    def request_state(self, state: MAIN_STATE):
        if state not in ALLOWED_TRANSITIONS[self.state]:
            self.state_requested = None
            raise ValueError(f"[Main] {state.name} cannot be reached from {self.state.name}!")
        log(f">> NEW STATE: {state}")
        # self.state_requested = state
    
    def move_state(self, state: MAIN_STATE) -> None:
        if state not in MAIN_STATE:
            raise ValueError(f"[Main] {state} invalid state value!")
        self.state = state
        self.state_requested = None
        
        if state == MAIN_STATE.INITIALIZED:
            self.kp = 0
            self.ki = 0
            self.kd = 0
            self.target_joint_torque = 0
            self.target_position = 0
            self.target_velocity = 0
        
        elif state == MAIN_STATE.SENSING_POSITION:
            self.kp = 0
            self.ki = 0
            self.kd = 0
            self.target_joint_torque = 0
            self.target_position = 0
            self.target_velocity = 0
            
        elif state == MAIN_STATE.MOVING_TO_ZERO:
            self.kp = P_GAIN_MOVE_TO_ZERO
            self.ki = I_GAIN_MOVE_TO_ZERO
            self.kd = D_GAIN_MOVE_TO_ZERO
            self.target_joint_torque = 0
            self.target_position = 0
            self.target_velocity = 0
            
        elif state == MAIN_STATE.ZERO_READY:
            self.kp = P_GAIN_HOLD
            self.ki = I_GAIN_HOLD
            self.kd = D_GAIN_HOLD
            self.target_joint_torque = 0
            self.target_position = 0
            self.target_velocity = 0
            
        elif state == MAIN_STATE.DAMPING:
            self.kp = P_GAIN_DAMPING
            self.ki = I_GAIN_DAMPING
            self.kd = D_GAIN_DAMPING
            self.target_joint_torque = 0
            self.target_position = 0
            self.target_velocity = 0
        
        else:
            self.kp = 0
            self.ki = 0
            self.kd = 0
            self.target_joint_torque = 0
            self.target_position = 0
            self.target_velocity = 0
            
    def zero_reached(self) -> bool:
        # later should be smarter to understand overflows etc.
        statusword, velocity, position_raw, torque = self.feedback
        position = position_raw / self.position_scale
        if (position + ZERO_MARGIN > 0) and (position - ZERO_MARGIN < 0):
            return True
        else:
            return False

    def status(self):
        statusword, velocity, position, torque = self.feedback
        return {
            "type": "status",
            "epos_state": self.get_epos_state_name(),
            "main_state": self.get_main_state_name(),
            "statusword": f"0x{statusword:04X}",
            "dq": velocity,
            "q": position / self.position_scale,
            "t": torque,
            "dq_target": self.target_velocity,
            "q_target": self.target_position,
            "t_target": self.target_joint_torque,
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "zero_reached": self.zero_reached(),
            "fault": bool(statusword & EPOS_STATE.MASK_FAULT),
        }

    def close(self):
        log("[EtherCat] Shutting down")
        if self.drive is not None and len(self.drive.output) == 26:
            try:
                for _ in range(100):
                    self.cycle(CW.SHUTDOWN)
                    time.sleep(CYCLE_S)
                for _ in range(100):
                    self.cycle(CW.DISABLE_VOLTAGE)
                    time.sleep(CYCLE_S)
            except Exception as ex:
                log(f"[EtherCat] Shutdown warning: {ex}")
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
        except Exception:
            pass
        self.master.close()


def handle_command(controller: JPVTController, command):
    name = command.get("command")
    if name == "status":
        return controller.status(), False
    if name == "enable":
        controller.enable()
        return {"type": "result", "command": name, "ok": True}, False
    if name == "state":
        state_cmd = command.get("state")
        if not isinstance(state_cmd, str):
            raise ValueError("state must be a string")
        state = state_cmd.strip().upper()
        if state == "DEVELOPER":
            controller.request_state(MAIN_STATE.MOVING_TO_ZERO)
        elif state == "DAMPING":
            controller.request_state(MAIN_STATE.DAMPING)
        else:
            raise ValueError(f"Invalid state request!")
        return {
            "type": "result",
            "command": name,
            "ok": True,
            "state_req": controller.state_requested,
            "state_curr": controller.get_main_state()
        }, False
    if name == "move":
        controller.move_target(
            command.get("q"), command.get("dq"), command.get("kp"), command.get("kd")
        )
        return {
            "type": "result", "command": name, "ok": True,
            "q_target": controller.target_position,
            "dq_target": controller.target_velocity,
            "t_target": controller.target_joint_torque,
            "kp": controller.kp,
            "kd": controller.kd,
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
    log(f"[Server] Listening on {SOCKET_PATH}")
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


def run_server(controller: JPVTController):
    server = open_server()
    client = None
    client_data = bytearray()
    quitting = False
    counter = 0
    motor_enabled = False
    try:
        while not quitting:
            cycle_start = time.monotonic()
            
            # EPOS state
            epos_state = controller.get_epos_state()
            if epos_state == EPOS_STATE.OPERATION_ENABLED:
                cw = CW.ENABLE_OPERATION
                motor_enabled = True
            else:
                cw = CW.DISABLE_VOLTAGE
                motor_enabled = False
                
            # MAIN state
            state = controller.get_main_state()
            state_requested = controller.state_requested
            
            if not motor_enabled:
                controller.move_state(MAIN_STATE.INITIALIZED)
                counter = 0
                
            else:
                if state == MAIN_STATE.INITIALIZED:
                    counter = 0
                    controller.move_state(MAIN_STATE.SENSING_POSITION)
                    
                elif state == MAIN_STATE.SENSING_POSITION:
                    counter += 1
                    if counter > N_CYCLES_SENSING_POSITION:
                        controller.move_state(MAIN_STATE.POSITION_READY)
                        counter = 0
                
                # elif state == MAIN_STATE.POSITION_READY:
                #     if state_requested == MAIN_STATE.MOVING_TO_ZERO:
                #         controller.move_state(state_requested)
                        
                elif state == MAIN_STATE.MOVING_TO_ZERO:
                    if controller.zero_reached():
                        counter += 1
                        if counter > N_CYCLES_ZERO_POSITION:
                            controller.move_state(MAIN_STATE.ZERO_READY)
                            counter = 0
                    else:
                        counter = 0
                
                # elif state == MAIN_STATE.ZERO_READY:
                #     if state_requested == MAIN_STATE.DAMPING:
                #         controller.move_state(state_requested)
                #         counter = 0
                
                elif state == MAIN_STATE.DAMPING:
                    pass
                
                
                if state != controller.state_requested and controller.state_requested is not None:
                    controller.move_state(state)
                
                # elif state == MAIN_STATE.SENSING_POSITION:
                #     counter += 1
                #     if counter > 1_000:
                #         controller.move_state(MAIN_STATE.DEVELOPER)
                #         counter = 0
                
                # elif state == MAIN_STATE.DEVELOPER:
                        
                #     # get to zero position slowly
                #     if controller.zero_reached():
                #         counter += 1
                #         if counter > 500:
                #             controller.move_state(MAIN_STATE.DAMPING)
                #     else:
                #         counter = 0
                        
                # elif state == MAIN_STATE.DAMPING:
                #     # do whatever
                #     pass
                
                # else:
                #     controller.move_state(MAIN_STATE.INITIALIZED)
            
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
            if controller.get_epos_state() == EPOS_STATE.MASK_FAULT:
                raise RuntimeError(f"[EPOS] Drive fault: SW=0x{controller.feedback[0]:04X}")
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
        log("[Main] Interrupted")
    except Exception as ex:
        log(f"[Main] Fatal: {ex}")
    finally:
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)
        controller.close()

if __name__ == "__main__":
    main()
