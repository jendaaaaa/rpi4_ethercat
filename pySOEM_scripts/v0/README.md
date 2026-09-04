# IPC-based JPVT velocity control

Files:

- `ethercat_service.py`: owns PySOEM, EtherCAT, PDOs, and the drive state machine.
- `controller.py`: sends binary commands through a Unix datagram socket.
- `protocol.py`: contains the enums for commands, drive modes etc.

Run the service from ~:

```bash
sudo taskset -c 3 chrt -f 99 .venv/bin/python3 ethercat_service.py
```

Run the controller in another terminal:

```bash
.venv/bin/python3 controller.py
```

Controller commands:

```text
enable
vel 15000
vel 0
disable
shutdown
```

Recommended test order:

1. Start the service.
2. Send `enable`.
3. Wait until it prints `State: enabled`.
4. Send a small safe velocity first, for example `vel 500`.
5. Send `vel 0`.
6. Send `disable`.
7. Send `shutdown`.

The service uses motor ID `0` for the first EtherCAT slave.