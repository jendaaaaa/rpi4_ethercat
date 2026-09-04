# DEVNOTES
Place for notes about making the EtherCAT work with HEJ maxon system.

## Entries
### JANUARY - APRIL
- HEJ connected to the battery. Using Maxon EPOS studio, the *EtherCAT* communication was established.
- EPOS Studio wanted a binary file for the GUI, the latest one `154h` was chosen.
- The state was changed to `enabled` based on the video provided by *Maxon*.
- HEJ spun for few seconds with a bit out of tune manner, but still moved!

### MAY
#### x.5.
- Raspberry Pi was connected to HEJ. Using a Windows machine running WSL connected to RPi through SSH, the `slaveinfo` script was executed successfully. Slave info was found.
- Using EPOS Studio, ESI file was obtained and stored on cloud.

#### 28.5.
- Successfully found HEJ using PySOEM script.
- Successfully changed mode of operation to `[9] CSV` from `[-64] JPVT` (default).
- Successfully changed state to Operational (enabled) and the motor was rumbling. But not spinning. Possible reasons:
    - DC (distributed clock) - Python is running slow or not consistent intervals, so the EPOS4 is confused.
    - The scaling of the Target velocity value is off so the HEJ is commanded to spin very slowly (near zero) even when set to 150.

### JUNE
#### 3.6.
- Again tested the EPOS studio approach, which worked and PySOEM still didnt (only humming sound and confirmed OP state, but no movement).
- After going through all the parameters in the PDO with HEJ unplugged, the Target Velocity formatted in a different way, from 5.000rpm to just raw value 5000, therefore the whole problem was setting too low Target Velocity.
- After setting the Target Velocity to 5000 instead of 5, the motor moved!!
- Also made the PVM work. Now i need to create some kind of framework for the PySOEM and specific firmware of HEJ to make a good state machine core made of modules to then use it for predefined movement sequences.

#### 9.6.
- Verified the JPVT mode. To control the velocity, set the JPVTC controller D gain to around `10_000`. Then set **Target Velocity** to `10_000` as well. Then set the `controlword` to `0x0006` and then `0x000F` and it spins!
- To control position, set the P gain to around `0.5` (maybe 500 -> needs to be verified in EPOS Studio), the rest is the same. Then keep changing the **Target Position** to values with step of around `400`.
- The velocity control in JPVT mode was tested using **EPOS Studio** and also the **PySOEM from RPi**.
- The position control in JPVT mode was tested using **EPOS Studio** only.

#### 22.6.
- Creating a custom library for controlling multiple devices in JPVT mode in a desired motion like a sine wave.
- Deciding on the structure of the project. Ideas:
    - Stick with Python.
    - Create a simple codebase that will move the motor in a sine wave (pos) and record the commands, actual values and timestamps.

#### 17.7
- Working on the library, first making the JPVT velocity control work together with an IPC.
- Created v0 to try to replicate the run_JPVT.py script by controlling it from another script.

#### 3.9.
- New local scripts for reading the position, then also controlling the position from script!
- For testing, there is a new `run_jpvt_v2.py` in [test_scripts](../raspberry_os_gui/test_scripts/). It now has a correct anti-aliasing filter based on the EtherCAT frequency, also reads position, correctly scales the target and fusion position. Sets an absolute position calculated from relative offset and fusion position value.
- What to do next:
    - Add state machine and commands from a socket.
    - Add homing capability or something like that, but that might be part of the state machine - add `idle`, move the leg to desired position, set it as default, then use homing. Maybe copy mechanisms from existing projects.

#### 4.9.
- Few test versions of v0
- Working version [v0_3](../pySOEM_scripts/v0_3) that accepts commands through UNIX socket
- TODO:
    - add chaining
    - test latency