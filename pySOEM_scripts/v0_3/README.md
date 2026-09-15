# v0_3: JPVT socket controllers

This folder contains two versions of a local controller for one EPOS4/HEJ
joint in JPVT mode. Start one controller on the Raspberry Pi, then use its
client from another terminal or Python script on the same machine.

| Pair | Gain handling | EtherCAT PDO sizes |
| --- | --- | --- |
| `controller.py` + `client.py` | P, I and D are configured once at startup through SDO | Rx: 10 bytes; Tx: 10 bytes |
| `controller_2.py` + `client_2.py` | Optional P and D values on relative moves; stored P, I and D are sent every cycle through PDO | Rx: 26 bytes; Tx: 14 bytes |

Both use a Unix socket at `/tmp/jpvt.sock`. Run only one controller at a time:
these versions share both the socket path and the EtherCAT interface. This
communication is local to the Raspberry Pi; it does not use DDS or a network
socket.

## Common behavior

Both controllers use PySOEM, the first discovered slave on `eth0`, JPVT mode
`-64`, and a requested EtherCAT cycle period of 2 ms. The Python loop sleeps
for the remaining time after each cycle; 2 ms is a requested period rather
than a guaranteed deadline.

At startup, the controller:

1. Finds the drive and enters EtherCAT PREOP.
2. Writes controlword zero and resets a startup fault only if the Fault bit is set.
3. Selects JPVT mode and writes the default gains, zero target torque and zero target velocity.
4. Reads the position units and initializes the target from the measured position.
5. Sets the anti-alias cutoff to 250 Hz and interpolation period to 2 ms.
6. Configures PDOs, enters EtherCAT OP, and starts the socket server with drive power disabled.

EtherCAT OP means process data can be exchanged. Drive operation is enabled
separately using the `enable` command.

Both versions support these commands:

| Command | Behavior |
| --- | --- |
| `status` | Returns the latest feedback and stored targets as JSON |
| `enable` | Captures the measured position as the target, then runs the CiA 402 enable sequence |
| `move-relative N` | Sets a target equal to the latest measured position plus `N` whole increments; requires the drive to be enabled |
| `set-velocity N` | Stores an integer target velocity; zero selects no velocity feedforward |
| `disable` | Disables the drive and keeps the server running |
| `quit` | Replies, then attempts to disable the drive and closes the controller |

A relative move is a position step, with no ramp. It uses the measured
position at command handling time, not the previous target. It does not zero
the encoder or rewrite its origin.

Position targets must use increments (`0x60A8 = 0x00B50000`). Feedback can be
increments or milli-increments; the controller reads `0x34C6:0D` and divides
milli-increment feedback by 1000 before calculating a target. No conversion
to radians is performed.

Velocity commands are passed directly to `0x60FF` without conversion. In the
HEJ configuration used here, these values are milli-rpm: `10000` means
10 rpm. The scripts do not read or validate the velocity unit setting.

Targets remain stored between client commands and are sent every EtherCAT
cycle. Clients send one command, read one response, and disconnect; they do
not need to send a heartbeat. There is no client-command timeout. `disable`
does not clear the stored target velocity, so set velocity to zero before
enabling if position-only control is intended.

## Original pair: controller.py and client.py

Start the controller from this folder:

```bash
run_realtime controller.py
```

Leave that terminal running. From another terminal:

```bash
python3 client.py status
python3 client.py set-velocity 0
python3 client.py enable
python3 client.py move-relative 300
python3 client.py move-relative -300
python3 client.py status
python3 client.py disable
python3 client.py quit
```

`run_realtime` is the existing launcher used on the Raspberry Pi. PySOEM must
be installed in the Python environment used by that launcher.

The gains are the constants at the top of `controller.py`:

```python
P_GAIN = 50_000
I_GAIN = 0
D_GAIN = 10_000
```

These constants are already in HEJ integer units. P is mNm/rad and D is
mNm·s/rad, so these defaults correspond to P = 50 Nm/rad and D = 10 Nm·s/rad.
The client cannot override them. Change the constants and restart the
controller to use other defaults.

The RxPDO contains controlword, target velocity and target position. The
TxPDO contains statusword, filtered velocity and filtered position. Torque
is cleared through SDO at startup; gains are also written through SDO at
startup rather than transmitted in each PDO.

Example `status` response:

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

## Extended pair: controller_2.py and client_2.py

Start the extended controller instead of the original:

```bash
run_realtime controller_2.py
```

Use the matching client:

```bash
python3 client_2.py status
python3 client_2.py set-velocity 0
python3 client_2.py enable
python3 client_2.py move-relative 300 kp 40 kd 1
python3 client_2.py status
python3 client_2.py disable
python3 client_2.py quit
```

The extended client adds optional `kp VALUE` and `kd VALUE` pairs to
`move-relative`. Either can appear first:

```bash
python3 client_2.py move-relative 300
python3 client_2.py move-relative 300 kp 40
python3 client_2.py move-relative 300 kd 1
python3 client_2.py move-relative 300 kd 1 kp 40
python3 client_2.py move-relative -300 kp 0.1 kd 1
```

Command values use SI units. The controller converts them to the HEJ's
unsigned 32-bit integer gains using `round(value * 1000)`:

| Parameter | Client unit | HEJ unit | Example |
| --- | --- | --- | --- |
| `kp` | Nm/rad | mNm/rad | `kp 40` becomes `40000` |
| `kd` | Nm·s/rad | mNm·s/rad | `kd 1` becomes `1000` |

Decimal command values are accepted; for example, `kp 0.1` becomes `100`.
The stored HEJ value has integer resolution. Negative, non-finite or
out-of-range gain values are rejected.

**Every relative-move command restores any omitted gain to its default.**
It does not keep an override from the previous move:

```bash
python3 client_2.py move-relative 300 kp 40 kd 1
# P=40000, D=1000 in HEJ units

python3 client_2.py move-relative 300 kp 40
# P=40000, D=D_GAIN (10000)

python3 client_2.py move-relative 300
# P=P_GAIN (50000), D=D_GAIN (10000)
```

The defaults are the constants in `controller_2.py`, in HEJ integer units:

```python
P_GAIN = 50_000
I_GAIN = 0
D_GAIN = 10_000
```

I remains at `I_GAIN`; there is no `ki` command. The optional gain arguments
are supported only by `move-relative`, not by `enable` or `set-velocity`.
Other commands do not change the stored gains.

Each RxPDO sends the latest stored values in this order:

```text
controlword, target velocity, target position, target torque, P, I, D
```

The gains do not need to change every cycle: the controller repeatedly sends
their stored values, even when no client is connected. Target torque is zero
by default and has no client command in this version.

Each TxPDO returns:

```text
statusword, filtered velocity, filtered position, filtered estimated joint torque
```

The extended PDO mapping must be supported by the drive firmware. Startup
validates a 26-byte RxPDO and a 14-byte TxPDO.

The extended status response adds these fields to the original response:

```json
{
  "torque_mNm": 0,
  "kp_mNm_per_rad": 40000,
  "kd_mNm_s_per_rad": 1000
}
```

These are additional fields, not a complete response. The gain fields report
the controller's stored command values; they are not separate SDO readbacks
from the drive. The relative-move result also reports the target and the
stored P and D gains.

## Sending commands from another Python script

Import the client corresponding to the running controller:

```python
from client import send_command

print(send_command({"command": "status"}))
print(send_command({"command": "set_velocity", "velocity": 0}))
print(send_command({"command": "enable"}))
print(send_command({"command": "move_relative", "increments": 300}))
print(send_command({"command": "disable"}))
```

For the extended version:

```python
from client_2 import send_command

print(send_command({"command": "status"}))
print(send_command({"command": "set_velocity", "velocity": 0}))
print(send_command({"command": "enable"}))
print(send_command({
    "command": "move_relative",
    "increments": 300,
    "kp": 40.0,
    "kd": 1.0,
}))
print(send_command({"command": "disable"}))
```

The wire format is newline-terminated JSON. Python/JSON command names use
underscores (`move_relative`, `set_velocity`); the command-line clients use
hyphens (`move-relative`, `set-velocity`). An invalid command returns a result
with `"ok": false` and an `"error"` description. Normal command results contain
`"ok": true`; `status` returns a status object directly.

## Logs and shutdown

Controller setup, errors and shutdown messages go to stderr. Client JSON
responses go to stdout. No log file is created automatically.

Save the extended controller log:

```bash
run_realtime controller_2.py 2>jpvt.log
```

For the original version, substitute `controller.py`.

Both controllers attempt the drive disable sequence on `quit`, Ctrl-C or a
fatal error, then request EtherCAT INIT and close the master. A drive fault,
non-positive receive WKC or unexpected feedback size causes a fatal error.
Shutdown can fail if EtherCAT communication is already lost; failures are
logged as shutdown warnings.

The socket is removed during normal cleanup. If it remains after an abrupt
termination, the next controller start removes the old socket path before
listening. Stop the previous controller before starting either version.
