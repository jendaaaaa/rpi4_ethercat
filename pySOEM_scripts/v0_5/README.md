# v0_5: Position diagnostics

`read_positions.py` continuously reads position-related SDO objects from the
first EPOS4 slave. It does not enable the drive, change gains or targets, or
configure PDOs. EtherCAT discovery places the slave in PREOP for mailbox
communication. This is a diagnostic reader, not an emergency stop or a
replacement for disabling the drive.

## Run

First disable the drive and quit any running controller. Only one PySOEM
master should own `eth0`; do not run this alongside `controller.py` or another
EtherCAT controller.

From this folder, using your existing launcher and PySOEM environment:

```bash
run_realtime read_positions.py
```

Or specify the interface and polling interval:

```bash
sudo /path/to/.venv/bin/python3 read_positions.py eth0 --interval 0.5
```

It polls until Ctrl-C. Move the disabled joint by hand and compare the columns.
No socket client is needed. The existing `controller.py` and `client.py` are
separate files and were not changed for this test.

## Readings

Configuration objects are printed once at startup:

| Object | Meaning |
| --- | --- |
| `0x3000:01` | Sensor configuration |
| `0x3000:02` | Control structure |
| `0x3012:02` | Configured SSI frame bits |
| `0x3012:0B` | SSI position bit count |
| `0x3012:08` | SSI power-up time in ms |
| `0x60A8:00` | Drive position units |
| `0x34C6:0D` | Sensor-fusion position units |

The table continuously reads:

| Column | Object | Meaning |
| --- | --- | --- |
| SSI raw | `0x3012:09` | Unsigned raw absolute SSI encoder position, increments |
| Actual | `0x6064:00` | Signed drive position actual value |
| Fused raw | `0x34C6:06` | Signed sensor-fusion position |
| Filtered raw | `0x34C6:0A` | Signed filtered sensor-fusion position |
| Target | `0x607A:00` | Signed drive target position |
| SW | `0x6041:00` | Statusword in hexadecimal |

The script deliberately prints raw position values without assuming the
control reference matches the raw encoder reference. If fusion units are
`0xFDB50000`, divide fused/filtered readings by 1000 to get increments. If
units are `0x00B50000`, they are already increments. In the documented
12-bit single-turn configuration, SSI raw covers one revolution with 4096
increments and wraps at its boundary.

If SSI raw changes with hand movement while fusion does not, that identifies
a difference between encoder acquisition and the position used by JPVT. It
does not by itself establish the correct target offset or safe enable sequence.

## Power-off tests and logging

A failed read is displayed as `unavailable`, with the exception type and
message printed when the error changes. It is never substituted with zero
or a previous successful position. Polling continues so temporary failures
can recover. A complete power cycle may require restarting this script if the
slave no longer supports mailbox reads; it does not rediscover or reconfigure
the drive automatically.

Save output to a file and display it:

```bash
run_realtime read_positions.py 2>&1 | tee positions.log
```

For comparing power cycles, stop the script, remove power, move the joint,
restore power, then start the script again. This keeps discovery explicit and
the drive unenabled throughout the diagnostic procedure.
