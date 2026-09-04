import enum
import struct

SOCKET_PATH = "/tmp/maxon_ethercat.sock"

# command:uint8, motor_id:uint8, value:int32
PACKET = struct.Struct("<BBi")

class DriveCommand(enum.IntEnum):
    ENABLE          = 1
    SET_VELOCITY    = 2
    DISABLE         = 3
    SHUTDOWN        = 4
    STATUS          = 5

class DriveMode(enum.IntEnum):
    PP      = 1     # Profile Position
    PV      = 3     # Profile Velocity
    H       = 6     # Homing
    CSP     = 8     # Cyclic Synchronous Position
    CSV     = 9     # Cyclic Synchronous Velocity
    CST     = 10    # Cyclic Synchronous Torque
    CSTCA   = 11    # Cyclic Synchronous Torque Mode with Commutation Angle [Internal Test Only]
    CLOI    = -60   # Cogging Compensation Position Offset Identification
    JF      = -62   # Joint Freeze
    JPVT    = -64   # Joint Position Velocity Torque

def pack_command(command: DriveCommand, motor_id: int = 0, value: int = 0,) -> bytes:
    return PACKET.pack(int(command), motor_id, value)

def unpack_command(data: bytes) -> tuple[DriveCommand, int, int]:
    command_value, motor_id, value = PACKET.unpack(data)

    return (DriveCommand(command_value), motor_id, value)