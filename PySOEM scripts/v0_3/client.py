"""Send one command to jpvt_json_process.py and print its response."""

import argparse
import json
import socket


SOCKET_PATH = "/tmp/jpvt.sock"


def send_command(command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.connect(SOCKET_PATH)
        client.sendall(json.dumps(command).encode() + b"\n")

        data = bytearray()
        while b"\n" not in data:
            chunk = client.recv(4096)
            if not chunk:
                raise RuntimeError("Controller closed without a response")
            data.extend(chunk)

    return json.loads(bytes(data).split(b"\n", 1)[0])


def parse_arguments():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("enable")
    commands.add_parser("status")
    commands.add_parser("disable")
    commands.add_parser("quit")

    move = commands.add_parser("move-relative")
    move.add_argument("increments", type=int)

    velocity = commands.add_parser("set-velocity")
    velocity.add_argument("velocity", type=int)

    return parser.parse_args()


def main():
    args = parse_arguments()
    command = {"command": args.command.replace("-", "_")}
    if args.command == "move-relative":
        command["increments"] = args.increments
    elif args.command == "set-velocity":
        command["velocity"] = args.velocity

    print(json.dumps(send_command(command), indent=2))


if __name__ == "__main__":
    main()
