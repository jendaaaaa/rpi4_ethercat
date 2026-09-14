"""Bridge Unitree-compatible DDS messages to one EPOS4 HEJ drive."""

import dataclasses
import sys
import threading
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from controller import CYCLE_S, INTERFACE, JPVTController


COMMAND_TOPIC = "rt/epos4/lowcmd"
STATE_TOPIC = "rt/epos4/lowstate"
MOTOR_INDEX = 0
COMMAND_TIMEOUT_S = 0.1


@dataclasses.dataclass
class MotorCommand:
    mode: int
    q: float
    dq: float
    kp: float
    kd: float
    tau: float


class DDSBridge:
    def __init__(self, controller):
        self.controller = controller
        self.crc = CRC()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.command = None
        self.command_time = 0.0
        self.command_lock = threading.Lock()
        self.tick = 0

        self.publisher = ChannelPublisher(STATE_TOPIC, LowState_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber(COMMAND_TOPIC, LowCmd_)
        self.subscriber.Init(self._command_callback, 1)

    def _command_callback(self, message):
        motor = message.motor_cmd[MOTOR_INDEX]
        command = MotorCommand(
            mode=motor.mode,
            q=motor.q,
            dq=motor.dq,
            kp=motor.kp,
            kd=motor.kd,
            tau=motor.tau,
        )
        with self.command_lock:
            self.command = command
            self.command_time = time.monotonic()

    def _latest_command(self):
        with self.command_lock:
            return self.command, self.command_time

    def apply_command(self):
        command, command_time = self._latest_command()
        command_is_fresh = (
            command is not None
            and time.monotonic() - command_time <= COMMAND_TIMEOUT_S
        )

        if not command_is_fresh:
            if self.controller.state() == "operation_enabled":
                self.controller.target_velocity = 0
                self.controller.target_torque = 0
                self.controller.disable()
            return

        if command.mode == 0:
            if self.controller.state() == "operation_enabled":
                self.controller.disable()
            return

        if command.mode != 1:
            raise ValueError(f"Unsupported motor mode: {command.mode}")

        if self.controller.state() != "operation_enabled":
            self.controller.capture_origin(command.q)

        self.controller.apply_unitree_command(
            command.q, command.dq, command.kp, command.kd, command.tau
        )

        if self.controller.state() != "operation_enabled":
            self.controller.enable()

    def publish_state(self):
        q, dq, tau = self.controller.unitree_state()
        motor = self.low_state.motor_state[MOTOR_INDEX]
        motor.mode = 1 if self.controller.state() == "operation_enabled" else 0
        motor.q = q
        motor.dq = dq
        motor.tau_est = tau

        self.tick = (self.tick + 1) & 0xFFFFFFFF
        self.low_state.tick = self.tick
        self.low_state.crc = self.crc.Crc(self.low_state)
        self.publisher.Write(self.low_state)


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: sudo {sys.argv[0]} <DDS network interface>")

    dds_interface = sys.argv[1]
    if dds_interface == INTERFACE:
        raise SystemExit(
            f"DDS cannot use {INTERFACE}; that interface is reserved for EtherCAT"
        )

    ChannelFactoryInitialize(0, dds_interface)
    controller = JPVTController(INTERFACE)

    try:
        controller.configure()
        bridge = DDSBridge(controller)
        print(
            f"DDS ready on {dds_interface}: {COMMAND_TOPIC} -> {STATE_TOPIC}",
            flush=True,
        )

        while True:
            cycle_start = time.monotonic()
            controller.run_cycle()
            bridge.apply_command()
            bridge.publish_state()

            remaining = CYCLE_S - (time.monotonic() - cycle_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()


if __name__ == "__main__":
    main()
