from dataclasses import dataclass

import pysoem

JOINT_TORQUE_LIMIT_MNM = 1000

@dataclass
class JointFeedback:
    statusword: int = 0
    q: int = 0
    dq: int = 0
    tau: int = 0

@dataclass
class JointCommand:
    controlword: int = 0
    q: int = 0
    dq: int = 0
    tau: int = 0
    kp: int = 0
    ki: int = 0
    kd: int = 0

@dataclass
class DriveConfig:
    name: str
    index: int
    direction: int = 1
    zero_offset_inc: int = 0
    incs_per_rev: int = 4096
    max_torque_mnm: int = 1000

# One Joint / Drive / EPOS4 device

class DriveEPOS4:
    def __init__(self, slave, name:str, index: int):
        self.slave = slave
        
        self.config = DriveConfig(name=name, index=index)
        self.command = JointCommand()
        self.feedback = JointFeedback()
        
    def config(self):
        pass
        






class EthercatChain:
    def __init__(self, interface: str = "eth0"):
        self.master = pysoem.Master()
        
        self.master.open(interface)