import socket

from protocol import (DriveCommand, SOCKET_PATH, pack_command)

MOTOR_ID = 0

def send_command(sock, command: int, value: int = 0) -> None:
    packet = pack_command(command=command, motor_id=0, value=value)
    sock.sendto(packet, SOCKET_PATH)

def main() -> None:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    print("Commands:")
    print("  enable")
    print("  vel <rpm>")
    print("  disable")
    print("  status")
    print("  shutdown")
    print("  quit")

    while True:
        parts = input("> ").strip().lower().split()

        if not parts:
            continue

        try:
            if parts[0] == "enable":
                send_command(sock, DriveCommand.ENABLE)

            elif parts[0] == "vel" and len(parts) == 2:
                send_command(sock, DriveCommand.SET_VELOCITY, int(parts[1]))

            elif parts[0] == "disable":
                send_command(sock, DriveCommand.DISABLE)
            
            elif parts[0] == "status":
                send_command(sock, DriveCommand.STATUS)

            elif parts[0] == "shutdown":
                send_command(sock, DriveCommand.SHUTDOWN)
                break

            elif parts[0] == "quit":
                break

            else:
                print("Unknown command")

        except FileNotFoundError:
            print("EtherCAT service is not running")
        except ValueError:
            print("Velocity must be an integer RPM value")

    sock.close()

if __name__ == "__main__":
    main()