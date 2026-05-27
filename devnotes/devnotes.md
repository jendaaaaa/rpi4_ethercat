# DEVNOTES
Place for notes about making the EtherCAT work with HEJ maxon system.

## Entries
### Before May
- HEJ connected to the battery. Using Maxon EPOS studio, the *EtherCAT* communication was established.
- EPOS Studio wanted a binary file for the GUI, the latest one `154h` was chosen.
- The state was changed to `enabled` based on the video provided by *Maxon*.
- HEJ spun for few seconds with a bit out of tune manner, but still moved!

### May
- Raspberry Pi was connected to HEJ. Using a Windows machine running WSL connected to RPi through SSH, the `slaveinfo` script was executed successfully. Slave info was found.
- Using EPOS Studio, ESI file was obtained and stored on cloud.