from __future__ import annotations

import enum
import os
import signal
import socket
import struct
import time

import pysoem

from protocol import (DriveCommand, DriveMode, PACKET, SOCKET_PATH, unpack_command)

INTERFACE = "eth0"

CYCLE_S = 0.002
D_GAIN = 10_000

class DriveState(enum.Enum):
    DISABLED = "disabled"
    ENABLING_06 = "enabling_06"
    ENABLING_0F = "enabling_0F"
    ENABLED = "enabled"
    DISABLING_ZERO = "disabling_zero"
    DISABLING_06 = "disabling_06"
    FAULT = "fault"

class EtherCATDriveService:
    def __init__(self, interface: str = INTERFACE) -> None:
        self.interface = interface
        self.master = pysoem.Master()
        self.drive = None
        self.sock = None

        self.running = True
        self.state = DriveState.DISABLED
        self.state_cycles = 0

        self.target_velocity = 0
        self.statusword = 0
        self.filtered_velocity = 0
        
        self.drive_mode = DriveMode.JPVT.value

    def run(self) -> None:
        self._open_ipc()
        self._configure_ethercat()

        print(f"IPC socket ready: {SOCKET_PATH}")
        print("Drive service ready. State: disabled")

        next_cycle = time.monotonic()

        try:
            while self.running:
                next_cycle += CYCLE_S

                self._read_commands()
                controlword, velocity = self._state_machine_step()

                self.statusword, self.filtered_velocity = self._cycle(controlword, velocity)

                if self.state == DriveState.ENABLED and self.statusword & 0x0008:
                    print(f"Drive fault detected, SW={hex(self.statusword)}")
                    self._set_state(DriveState.FAULT)

                delay = next_cycle - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_cycle = time.monotonic()

        finally:
            self.close()

    def _open_ipc(self) -> None:
        try:
            os.unlink(SOCKET_PATH)
        except FileNotFoundError:
            pass

        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(SOCKET_PATH)
        self.sock.setblocking(False)

    def _read_commands(self) -> None:
        while True:
            try:
                data = self.sock.recv(PACKET.size)
            except BlockingIOError:
                return

            if len(data) != PACKET.size:
                continue

            try:
                command, motor_id, value = unpack_command(data)
            except ValueError:
                print("Unknown command received")
                continue

            # One drive for now. Reserve motor_id 0 for the first drive.
            if motor_id != 0:
                continue

            if command == DriveCommand.ENABLE:
                if self.state == DriveState.DISABLED:
                    self._set_state(DriveState.ENABLING_06)

            elif command == DriveCommand.SET_VELOCITY:
                self.target_velocity = value
                print(f"Target velocity: {value} rpm")

            elif command == DriveCommand.DISABLE:
                if self.state in {
                    DriveState.ENABLING_06,
                    DriveState.ENABLING_0F,
                    DriveState.ENABLED,
                }:
                    self._set_state(DriveState.DISABLING_ZERO)

            elif command == DriveCommand.SHUTDOWN:
                self.running = False
                
            elif command == DriveCommand.STATUS:
                print(
                    f"State={self.state.value}, "
                    f"mode={DriveMode(self.drive_mode).name}, "
                    f"statusword={hex(self.statusword)}, "
                    f"target_velocity={self.target_velocity}, "
                    f"filtered_velocity={self.filtered_velocity}"
                )

    def _set_state(self, new_state: DriveState) -> None:
        if new_state != self.state:
            self.state = new_state
            self.state_cycles = 0
            print(f"State: {new_state.value}")

    def _state_machine_step(self) -> tuple[int, int]:
        self.state_cycles += 1

        if self.state == DriveState.DISABLED:
            return 0x0000, 0

        if self.state == DriveState.ENABLING_06:
            if self.state_cycles >= 250:
                self._set_state(DriveState.ENABLING_0F)
            return 0x0006, 0

        if self.state == DriveState.ENABLING_0F:
            if (self.statusword & 0x006F) == 0x0027:
                self._set_state(DriveState.ENABLED)
            elif self.state_cycles >= 500:
                print(f"Enable failed, SW={hex(self.statusword)}")
                self._set_state(DriveState.FAULT)
            return 0x000F, 0

        if self.state == DriveState.ENABLED:
            return 0x000F, self.target_velocity

        if self.state == DriveState.DISABLING_ZERO:
            if self.state_cycles >= 250:
                self._set_state(DriveState.DISABLING_06)
            return 0x000F, 0

        if self.state == DriveState.DISABLING_06:
            if self.state_cycles >= 100:
                self._set_state(DriveState.DISABLED)
            return 0x0006, 0

        return 0x0000, 0

    def _configure_ethercat(self) -> None:
        self.master.open(self.interface)

        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found")

        self.drive = self.master.slaves[0]
        print(f"Found slave: {self.drive.name}")

        self.master.state = pysoem.PREOP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.PREOP_STATE, timeout=50_000)
        print("EtherCAT state: PREOP")

        write_i8(self.drive, 0x6060, 0, self.drive_mode)
        time.sleep(0.1)

        mode = read_i8(self.drive, 0x6061, 0)
        print(f"Mode display 0x6061: {mode}")

        if mode != DriveMode.JPVT.value:
            raise RuntimeError("JPVT mode was not accepted")

        write_u32(self.drive, 0x34C6, 1, 0)
        write_u32(self.drive, 0x34C6, 2, 0)
        write_u32(self.drive, 0x34C6, 3, D_GAIN)

        try:
            self.drive.sdo_write(0x34C3, 0, b"\x00\x00\x00\x00")
        except Exception as exc:
            print(f"Could not clear 0x34C3, continuing: {exc}")

        write_i32(self.drive, 0x60FF, 0, 0)

        self.drive.sdo_write(0x60C2, 1, struct.pack("<B", 2))
        self.drive.sdo_write(0x60C2, 2, struct.pack("<b", -3))

        self._configure_pdos()
        self.master.config_map()

        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.SAFEOP_STATE, timeout=50_000)
        print("EtherCAT state: SAFEOP")

        print(f"RxPDO size: {len(self.drive.output)} bytes")
        print(f"TxPDO size: {len(self.drive.input)} bytes")

        self.master.state = pysoem.OP_STATE
        self.master.write_state()

        print("Feeding watchdog...")
        for _ in range(300):
            self._cycle(0x0000, 0)
            time.sleep(CYCLE_S)

        self.master.read_state()
        if self.drive.state != pysoem.OP_STATE:
            raise RuntimeError(
                f"Could not enter OP state. State: {self.drive.state}"
            )

        print("EtherCAT state: OP")

    def _configure_pdos(self) -> None:
        drive = self.drive

        drive.sdo_write(0x1C12, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1C13, 0, struct.pack("<B", 0))

        drive.sdo_write(0x1603, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1603, 1, struct.pack("<I", 0x60400010))
        drive.sdo_write(0x1603, 2, struct.pack("<I", 0x60FF0020))
        drive.sdo_write(0x1603, 0, struct.pack("<B", 2))

        drive.sdo_write(0x1A03, 0, struct.pack("<B", 0))
        drive.sdo_write(0x1A03, 1, struct.pack("<I", 0x60410010))
        drive.sdo_write(0x1A03, 2, struct.pack("<I", 0x34C60B20))
        drive.sdo_write(0x1A03, 0, struct.pack("<B", 2))

        drive.sdo_write(0x1C12, 1, struct.pack("<H", 0x1603))
        drive.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A03))

        drive.sdo_write(0x1C12, 0, struct.pack("<B", 1))
        drive.sdo_write(0x1C13, 0, struct.pack("<B", 1))

    def _cycle(self, controlword: int, target_velocity: int) -> tuple[int, int]:
        self.drive.output = struct.pack(
            "<Hi",
            int(controlword),
            int(target_velocity),
        )

        self.master.send_processdata()
        self.master.receive_processdata()

        if len(self.drive.input) >= 6:
            return struct.unpack("<Hi", self.drive.input[:6])

        return 0, 0

    def close(self) -> None:
        print("\nShutting down EtherCAT service...")

        try:
            if self.drive is not None:
                for _ in range(250):
                    self._cycle(0x000F, 0)
                    time.sleep(CYCLE_S)

                # for _ in range(100):
                #     self._cycle(0x0006, 0)
                #     time.sleep(CYCLE_S)

                for _ in range(100):
                    self._cycle(0x0000, 0)
                    time.sleep(CYCLE_S)

                self.master.state = pysoem.INIT_STATE
                self.master.write_state()

        except Exception as exc:
            print(f"Shutdown warning: {exc}")

        try:
            self.master.close()
        except Exception:
            pass

        if self.sock is not None:
            self.sock.close()

        try:
            os.unlink(SOCKET_PATH)
        except FileNotFoundError:
            pass

        print("Done.")


def write_i8(drive, index: int, subindex: int, value: int) -> None:
    drive.sdo_write(index, subindex, struct.pack("<b", int(value)))


def read_i8(drive, index: int, subindex: int) -> int:
    return struct.unpack("<b", drive.sdo_read(index, subindex)[:1])[0]


def write_i32(drive, index: int, subindex: int, value: int) -> None:
    drive.sdo_write(index, subindex, struct.pack("<i", int(value)))


def write_u32(drive, index: int, subindex: int, value: int) -> None:
    drive.sdo_write(index, subindex, struct.pack("<I", int(value)))


def main() -> None:
    service = EtherCATDriveService()

    def stop_handler(signum, frame) -> None:
        service.running = False

    # handles CTRL+C or kill command
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    service.run()


if __name__ == "__main__":
    main()