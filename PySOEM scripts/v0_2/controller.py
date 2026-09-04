import time
import struct
import sys

import pysoem

INTERFACE = "eth0"

CYCLE_S = 0.002
N_CYCLES = 2_000

TARGET_VELOCITY_RPM = 0
POSITION_OFFSET = 500

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
        self.target_position = 0
        self.target_velocity = 0
        self.position_scale = 1
    
    def initialize(self):
        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found!")
        
        self.drive = self.master.slaves[0]
        log(f"Found slave: {self.drive.name}")
        
        if not self.drive:
            raise RuntimeError("Must initialize first!")
        
        self._activate_preop()
        self._reset_startup_fault()
        self._configure()
        self._activate_safeop()
        self._activate_op()
        
    def _activate_preop(self):
        self.master.state = pysoem.PREOP_STATE
        self.master.write_state()
        # self.master.state_check(pysoem.PREOP_STATE, timeout=50_000)
        if self.drive.state_check(pysoem.PREOP_STATE, timeout=50_000) != pysoem.PREOP_STATE:
            raise RuntimeError("Drive did not reach PREOP for startup recovery.")
        log("> State: PREOP")
        
    def _activate_safeop(self):
        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.SAFEOP_STATE, timeout=50_000)
        log("> State: SAFEOP")
    
    def _activate_op(self):
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        for _ in range(300):
            self.cycle()
            time.sleep(CYCLE_S)
            
        self.master.read_state()
        if self.drive.state != pysoem.OP_STATE:
            raise RuntimeError(f"Could not enter OP state. State: {self.drive.state}")
        
        log("State: OP")
        log(f"Ready: P={P_GAIN}, I={I_GAIN}, D={D_GAIN}")
    
    def _configure(self):
        # set JPVT mode
        self.write_i8(0x6060, 0, JPVT_MODE)
        time.sleep(0.1)
        
        mode = self.read_i8(0x6061, 0)
        if mode != JPVT_MODE:
            raise RuntimeError("JPVT mode was not accepted!")
        
        # set PID gains
        self.write_u32(0x34C6, 1, P_GAIN)
        self.write_u32(0x34C6, 2, I_GAIN)
        self.write_u32(0x34C6, 3, D_GAIN)
        log(
            "JPVT gains:",
            "P =", self.read_u32(0x34C6, 1),
            "I =", self.read_u32(0x34C6, 2),
            "D =", self.read_u32(0x34C6, 3),
        )
        
        # ------------------------------------------------------------
        # reset PVT values
        self.position_scale = self._read_position_scale()
        current_position = self.read_i32(0x34C6, 6)
        self.target_position = round(current_position / self.position_scale)
        
        self.write_i32(0x34C3, 0, 0) # torque
        self.write_i32(0x60FF, 0, 0) # velocity
        self.write_i32(0x607A, 0, self.target_position) # position
        log("PVT values reset.")
        
        # ------------------------------------------------------------
        # anti-aliasing cutoff
        cutoff_hz = round(1.0 / (2.0 * CYCLE_S))
        if not 1 <= cutoff_hz <= 0xFFFF:
            raise ValueError("Anti-alias cutoff must fit a positive 16-bit value.")
        self.drive.sdo_write(0x3676, 1, struct.pack("<H", cutoff_hz))
        actual_cutoff = struct.unpack("<H", self.drive.sdo_read(0x3676, 1))[0]
        if actual_cutoff != cutoff_hz:
            raise RuntimeError(f"Anti-alias cutoff readback mismatch: {actual_cutoff}, expected {cutoff_hz}")
        log(f"Anti-alias cutoff 0x3676:01: {actual_cutoff} Hz")
        
        # interpolation period
        self.drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        self.drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))
        
        # ------------------------------------------------------------
        # PDO mapping
        # assign PDOs to output Sync Managers
        self.drive.sdo_write(0x1C12, 0, struct.pack("<B", 0))
        self.drive.sdo_write(0x1C13, 0, struct.pack("<B", 0))
        
        # RxPDO: Controlword, Target Velocity, Target Position
        self.drive.sdo_write(0x1603, 0, struct.pack("<B", 0))
        self.drive.sdo_write(0x1603, 1, struct.pack("<I", 0x60400010))
        self.drive.sdo_write(0x1603, 2, struct.pack("<I", 0x60FF0020))
        self.drive.sdo_write(0x1603, 3, struct.pack("<I", 0x607A0020))
        self.drive.sdo_write(0x1603, 0, struct.pack("<B", 3))
        
        # TxPDO mapping: Statusword, filtered JPVT velocity, filtered position
        self.drive.sdo_write(0x1A03, 0, struct.pack("<B", 0))
        self.drive.sdo_write(0x1A03, 1, struct.pack("<I", 0x60410010))
        self.drive.sdo_write(0x1A03, 2, struct.pack("<I", 0x34C60B20))
        self.drive.sdo_write(0x1A03, 3, struct.pack("<I", 0x34C60A20))
        self.drive.sdo_write(0x1A03, 0, struct.pack("<B", 3))
        
        # Selecting mappings
        self.drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603))
        self.drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))
    
        # Assign PDOs to Sync Managers
        self.drive.sdo_write(0x1C12, 0, struct.pack("<B", 1))
        self.drive.sdo_write(0x1C13, 0, struct.pack("<B", 1))
        
        self.master.config_map()
        
        log(f"RxPDO size: {len(self.drive.output)} bytes, expected 10")
        log(f"TxPDO size: {len(self.drive.input)} bytes, expected 10")
    
    def _reset_startup_fault(self):
        statusword = struct.unpack("<H", self.drive.sdo_read(0x6041, 0))[0]
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
        if not (statusword & 0x0008):
            return

        log(f"Resetting startup fault: SW=0x{statusword:04X}")
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))
        time.sleep(0.2)
        self.drive.sdo_write(0x6040, 0, struct.pack("<H", 0x0000))
        time.sleep(0.2)
        if self.read_u16(self.drive, 0x6041, 0) & 0x0008:
            raise RuntimeError("Startup fault did not clear")
            
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
    
    def _read_position_scale(self):
        target_unit = self.read_u32(0x60A8, 0)
        fusion_unit = self.read_u32(0x34C6, 0x0D)
        if target_unit != 0x00B50000:
            raise RuntimeError(f"Unsupported target position unit: {target_unit:#010x}")
        if fusion_unit == 0x00B50000:
            return 1
        if fusion_unit == 0xFDB50000:
            return 1000
        raise RuntimeError(f"Unsupported feedback position unit: {fusion_unit:#010x}")
    
    def enable(self):
        for controlword, expected_state in (
            (0x0006, 0x0021),  # Ready to switch on
            (0x0007, 0x0023),  # Switched on
            (0x000F, 0x0027),  # Operation enabled
        ):
            log(f"Controlword {controlword:#06x}")
            self._set_controlword(controlword, expected_state)

        log(f"\nRunning JPVT with target position {self.target_position}\n")
    
    def disable(self):
        self._set_controlword(0x0006, 0x0021)
        self._set_controlword(0x0000, 0x0040)
        
    def cycle(self, controlword):
        self.drive.output = struct.pack("<Hii", controlword, self.target_velocity, self.target_position)
        self.master.send_processdata()
        wkc = self.master.receive_processdata()
        if wkc <= 0:
            raise RuntimeError(f"EtherCAT receive failed: WKC={wkc}")
        if len(self.drive.input) != 10:
            raise RuntimeError(f"Expected 10 input bytes, received {len(self.drive.input)}")
        self.feedback = struct.unpack("<Hii", self.drive.input)
        return self.feedback
    
    def state(self):
        statusword = self.feedback[0]
        state = statusword & 0x006F
        return {
            0x0040: "switch_on_disabled",
            0x0021: "ready_to_switch_on",
            0x0023: "switched_on",
            0x0027: "operation_enabled",
        }.get(state, "fault" if statusword & 0x0008 else "unknown")
    
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
    
    # ------------------------------------------------------------
    def write_i8(self, index, subindex, value):
        self.drive.sdo_write(index, subindex, struct.pack("<b", int(value)))

    def read_i8(self, index, subindex):
        return struct.unpack("<b", self.drive.sdo_read(index, subindex)[:1])[0]

    def write_i32(self, index, subindex, value):
        self.drive.sdo_write(index, subindex, struct.pack("<i", int(value)))

    def read_i32(self, index, subindex):
        return struct.unpack("<i", self.drive.sdo_read(index, subindex)[:4])[0]

    def write_u32(self, index, subindex, value):
        self.drive.sdo_write(index, subindex, struct.pack("<I", int(value)))

    def read_u32(self, index, subindex):
        return struct.unpack("<I", self.drive.sdo_read(index, subindex)[:4])[0]