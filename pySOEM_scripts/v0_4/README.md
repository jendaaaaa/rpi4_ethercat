# v0.4 - Unitree-compatible DDS bridge

This version controls one EPOS4 HEJ over EtherCAT and exchanges Unitree HG
`LowCmd_` and `LowState_` messages over Cyclone DDS.

## Interfaces and topics

- EtherCAT uses `eth0`.
- DDS must use a different IP network interface, such as `wlan0` or `eth1`.
- Commands: `rt/epos4/lowcmd`
- State: `rt/epos4/lowstate`
- The EPOS4 is represented by `motor_cmd[0]` and `motor_state[0]`.

Dedicated topics prevent collisions with a Unitree robot using `rt/lowcmd` and
`rt/lowstate`. The message types and motor fields are the same.

## Confirmed conversions

| Unitree field | Unitree unit | EPOS4 raw unit | Conversion |
|---|---|---|---|
| `q` | rad | joint inc | `rad * 4096 / (2*pi)` |
| `dq` | rad/s | mrpm | `rad/s * 60000 / (2*pi)` |
| `kp` | Nm/rad | mNm/rad | `kp * 1000` |
| `kd` | Nm*s/rad | mNm*s/rad | `kd * 1000` |
| `tau` | Nm | mNm | `tau * 1000` |

The position conversion currently assumes the documented HW8A04 default:
12-bit SSI position, or 4096 joint increments/revolution. The controller reads
`0x3012:02` and logs a warning if it reports another single-turn resolution.

Position origins are captured when mode changes from disabled to enabled. The
first Unitree `q` command is associated with the current EPOS position, avoiding
a jump caused by unrelated absolute-zero references.

## Dependencies

Install `pysoem` on the Raspberry Pi. Install Unitree SDK2 Python with its
supported Cyclone DDS version on the Raspberry Pi and the remote DDS device.

## Run the bridge on the Raspberry Pi

The argument is the DDS/IP interface, not the EtherCAT interface:

```bash
run_realtime dds_controller.py wlan0
```

The bridge disables the drive if it receives no command for 100 ms.

## Run the example client

On the Raspberry Pi or another DDS device on the same network:

```bash
python3 client.py wlan0 --delta 0.15 --duration 3 --kp 50 --kd 1
```

This publishes a three-second position ramp of 0.15 rad. Press Ctrl-C to send
disabled mode. A zero-motion communication test is:

```bash
python3 client.py wlan0 --delta 0 --kp 0 --kd 0
```

Replace `wlan0` with the network-facing interface name on that device.

## Use from another device

1. Put both devices on the same IP network.
2. Install matching Unitree SDK2 Python and Cyclone DDS versions.
3. Start the bridge on the Raspberry Pi using its network-facing interface.
4. Start the client on the other device using that device's network-facing
   interface.
5. Ensure the network permits UDP multicast, which DDS discovery normally uses.

The other device does not connect to an IP address directly. DDS discovers the
bridge through the selected network interfaces. If multicast discovery is
blocked by the Wi-Fi access point, configure Cyclone DDS with explicit peers or
use a network that permits device-to-device multicast traffic.

## Message behavior

For `motor_cmd[0]`:

- `mode = 1` enables and commands the drive.
- `mode = 0` disables it.
- `q`, `dq`, `kp`, `kd`, and `tau` are applied continuously through the RxPDO.

The bridge publishes converted `q`, `dq`, and estimated `tau` in
`motor_state[0]` at the EtherCAT cycle rate.
