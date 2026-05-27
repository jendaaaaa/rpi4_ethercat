# DEVNOTES
Place for notes about making the EtherCAT work with HEJ maxon system.

## Entries
### Before May
- HEJ connected to the battery. Using Maxon EPOS studio, the *EtherCAT* communication was established.
- EPOS Studio wanted a binary file for the GUI, the latest one `154h` was chosen.
- The state was changed to `enabled` based on the video provided by *Maxon*.
- HEJ spun for few seconds with a bit out of tune manner, but still moved!

### May
#### Remote work
- Raspberry Pi was connected to HEJ. Using a Windows machine running WSL connected to RPi through SSH, the `slaveinfo` script was executed successfully. Slave info was found.
- Using EPOS Studio, ESI file was obtained and stored on cloud.

#### In Lab
- Successfully found HEJ using PySOEM script.
- Successfully changed mode of operation to `[9] CSV` from `[-64] JPVT` (default).
- Successfully changed state to Operational (enabled) and the motor was rumbling. But not spinning. Possible reasons:
    - DC (distributed clock) - Python is running slow or not consistent intervals, so the EPOS4 is confused.
    - The scaling of the Target velocity value is off so the HEJ is commanded to spin very slowly (near zero) even when set to 150.