# PySOEM Scripts and Custom Library
This section will explore the PySOEM library in more detail. A custom library to easily control the specific HEJ drive will be developed as well.

The script [run_jpvt.py](../raspberry_os_gui/test_scripts/run_jpvt.py) serves as an inspiration for how to control the HEJ drive. It successfully runs the drive and prints out the filtered joint velocity.

The RPi runnig the scripts is based on the RaspberryOS with the PREEMPT_RT, as previously described in [raspberry_os_gui](../raspberry_os_gui/).

To run any of the script, a custom shell script is used, as described in [raspberry_os_gui/README.md](../raspberry_os_gui/README.md).