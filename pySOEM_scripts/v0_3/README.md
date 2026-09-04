# JPVT Socket Controller

This pair of scripts keeps one EtherCAT controller process running and lets
other local processes send commands through a Unix socket.

## Files

- `jpvt_json_process.py` owns the EtherCAT interface and runs the 2 ms PDO loop.
- `jpvt_client.py` connects to the controller, sends one command, prints the
  response, and disconnects.

The controller listens on:

```text
/tmp/jpvt.sock
```

Run only one controller process. Multiple client invocations can connect to it
one at a time.

## Start the controller

From the directory containing both scripts:

```bash
run_realtime jpvt_json_process.py
```

Leave this terminal running. Setup and error messages are written to stderr.

To save the controller log:

```bash
run_realtime jpvt_json_process.py 2>jpvt.log
```

To display and save it:

```bash
run_realtime jpvt_json_process.py 2> >(tee -a jpvt.log >&2)
```

## Send commands

Use another terminal in the same directory.

Read the current state:

```bash
python3 jpvt_client.py status
```

Enable the drive. The controller captures the measured position and uses it as
the initial target before enabling:

```bash
python3 jpvt_client.py enable
```

Move relative to the measured position, in whole increments:

```bash
python3 jpvt_client.py move-relative 300
python3 jpvt_client.py move-relative -300
```

Set target velocity:

```bash
python3 jpvt_client.py set-velocity 10000
python3 jpvt_client.py set-velocity 0
```

Disable the drive while leaving the controller process running:

```bash
python3 jpvt_client.py disable
```

Disable, close EtherCAT, and stop the controller process:

```bash
python3 jpvt_client.py quit
```

## Typical position-control session

```bash
python3 jpvt_client.py status
python3 jpvt_client.py set-velocity 0
python3 jpvt_client.py enable
python3 jpvt_client.py move-relative 300
python3 jpvt_client.py status
python3 jpvt_client.py disable
```

## Status response

```json
{
  "type": "status",
  "state": "operation_enabled",
  "statusword": "0x1237",
  "position_inc": 72765.0,
  "target_inc": 73065,
  "target_velocity": 0,
  "velocity_raw": 0,
  "fault": false
}
```

`position_inc` and `target_inc` are in position increments. `target_velocity`
is the requested value, while `velocity_raw` is the feedback value received
from the drive.

## Use from Python

Other Python programs can import the client function:

```python
from jpvt_client import send_command

print(send_command({"command": "status"}))
print(send_command({"command": "enable"}))
print(send_command({"command": "move_relative", "increments": 300}))
print(send_command({"command": "set_velocity", "velocity": 0}))
print(send_command({"command": "disable"}))
```

Each `send_command()` call opens one socket connection, waits for one response,
and closes the connection.

## Controller parameters

The controller parameters are near the top of `jpvt_json_process.py`:

```python
CYCLE_S = 0.002
P_GAIN = 50_000
I_GAIN = 0
D_GAIN = 10_000
```

Target velocity starts at zero and is changed with the `set_velocity` command.

## Stopping after an interruption

The controller runs its disable sequence when it receives `quit`, when Ctrl-C
is pressed, or when it detects a fatal error. If the process terminates without
cleaning up, `/tmp/jpvt.sock` may remain. The next controller start removes the
old socket path before listening.
