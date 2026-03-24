# EtherCAT on Raspberry Pi 4b

## Setup

### 1. Raspberry Pi Imager
- Set up image of Ubuntu Server 24.04.4 LTS (64-bit)
- Configure WiFi name and password
- Enable SSH (i used login and password)

### 2. Turn on Raspberry Pi 4b
- Plug in SSD
- Power
- Wait for about 5 mins to go through initial setup

### 2.b Enahle SSH
- This step was unsuccessful, RPi can `ping google.com` but cannot ping my PC, also PC cannot ping RPi
- If you want to set up the rest remotely, enable SSH
`sudo systemclt status ssh` is probably **Loaded** but **Inactive**
`sudo systemctl enable --now ssh` should enable SSH, but i couldnt connect to it...

### 3. Run commands
- Update and Upgrade all packages
`sudo apt update`
`sudo apt upgrade -y`
- enable Ubuntu Pro (register at [ubuntu.com/pro](ubuntu.com/pro) and get the **Free Personal Token**)
- enable RealTime Kernel for Raspi
`sudo pro enable realtime-kernel --variant=raspi` and accept all `[Y]`

