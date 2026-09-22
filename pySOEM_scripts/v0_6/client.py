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
    
    state = commands.add_parser("state")
    state.add_argument("state")
    
    move = commands.add_parser("move")
    move.add_argument("q", type=int)
    move.add_argument("dq", type=int)
    move.add_argument("gains", nargs="*", metavar="GAIN VALUE")

    return parser, parser.parse_args()

def main():
    parser, args = parse_arguments()
    command = {"command": args.command.replace("-", "_")}
    if args.command == "state":
        command["state"] = args.state
    
    if args.command == "move":
        command["q"] = args.q
        command["dq"] = args.dq
        if len(args.gains) % 2:
            parser.error("gain parameters must be pairs, for example: kp 40 kd 1")
        for name, value in zip(args.gains[::2], args.gains[1::2]):
            if name not in ("kp", "kd"):
                parser.error(f"unknown gain {name!r}; use kp or kd")
            if name in command:
                parser.error(f"{name} was specified more than once")
            try:
                command[name] = float(value)
            except ValueError:
                parser.error(f"{name} must be a number")

    print(json.dumps(send_command(command), indent=2))

if __name__ == "__main__":
    main()