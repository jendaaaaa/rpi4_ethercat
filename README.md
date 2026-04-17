# [WIP] EtherCAT on Raspberry Pi 4b
This is my take on running EtherCAT master on Raspberry Pi 4b with a PREEMPT_RT patched kernel for Soft Real Time use. Different attempts are domunented below.

## Used hardware
Raspberry Pi side:
* **Raspberry Pi 4b** with microSD card
* USB-C cable and power adapter
* micro-HDMI to HDMI cable
* HDMI display
* USB-A keyboard

PC side:
* **MacBook M2 Pro**
* USB-C microSD card reader (dongle from Alza.cz)

Others:
* WiFi with internet access

## Attempts
* [Ubuntu Server 24.04.4 LTS](/ubuntu_server_pro/) with Ubuntu Pro (PREEMPT_RT included)
* [Raspberry OS Full](/raspberry_os_gui/) with recompiled Kernel for PREEMPT_RT