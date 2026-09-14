"""Small DDS motion example for the EPOS4 bridge."""

import argparse
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


COMMAND_TOPIC = "rt/epos4/lowcmd"
STATE_TOPIC = "rt/epos4/lowstate"
MOTOR_INDEX = 0
CONTROL_DT = 0.002


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="IP network interface, for example wlan0")
    parser.add_argument("--delta", type=float, default=0.0, help="position change [rad]")
    parser.add_argument("--duration", type=float, default=3.0, help="ramp time [s]")
    parser.add_argument("--kp", type=float, default=50.0, help="P gain [Nm/rad]")
    parser.add_argument("--kd", type=float, default=1.0, help="D gain [Nm*s/rad]")
    return parser.parse_args()


def main():
    args = arguments()
    ChannelFactoryInitialize(0, args.interface)

    state = {"message": None}

    def state_callback(message):
        state["message"] = message

    publisher = ChannelPublisher(COMMAND_TOPIC, LowCmd_)
    publisher.Init()
    subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
    subscriber.Init(state_callback, 1)

    while state["message"] is None:
        print("Waiting for EPOS4 state...")
        time.sleep(0.5)

    command = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()
    q_start = state["message"].motor_state[MOTOR_INDEX].q
    start_time = time.monotonic()

    try:
        while True:
            elapsed = time.monotonic() - start_time
            ratio = min(elapsed / args.duration, 1.0)
            motor = command.motor_cmd[MOTOR_INDEX]
            motor.mode = 1
            motor.q = q_start + args.delta * ratio
            motor.dq = 0.0
            motor.kp = args.kp
            motor.kd = args.kd
            motor.tau = 0.0
            command.crc = crc.Crc(command)
            publisher.Write(command)

            actual = state["message"].motor_state[MOTOR_INDEX]
            print(
                f"q={actual.q:9.5f} rad  dq={actual.dq:9.5f} rad/s  "
                f"target={motor.q:9.5f} rad",
                end="\r",
                flush=True,
            )
            time.sleep(CONTROL_DT)
    except KeyboardInterrupt:
        print()
    finally:
        # Publish disabled mode repeatedly so the bridge receives it.
        command.motor_cmd[MOTOR_INDEX].mode = 0
        for _ in range(20):
            command.crc = crc.Crc(command)
            publisher.Write(command)
            time.sleep(CONTROL_DT)


if __name__ == "__main__":
    main()
