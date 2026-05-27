# Raspberry Pi OS 64-bit Full
Attempt to run EtherCAT master on Raspberry Pi 4b using a Raspberry Pi OS (64-bit) Full with built Kernel and full GUI (can be turned off later for jitter reduction).

## Used hardware
Raspberry Pi side:
* **Raspberry Pi 4b** with microSD card
* USB-C cable and power adapter
* micro-HDMI to HDMI cable
* HDMI display
* USB-A keyboard

PC side:
* Windows 11 machine
* microSD to SD adapter

Others:
* WiFi with internet access 

---

## Before Installing
Prepare the image for a Raspberry Pi 4b.

### 1. Raspberry Pi Imager
* Set up image of Raspberry Pi OS (64 bit) Desktop
* Configure WiFi name and password.
* Enable SSH (i used login and password).

### 2. Start up Raspberry
Check if Raspberry is connected to the network and has internet access.

---

## Installation
### 1. Connect and prepare libraries
* Video tutorial i followed can be found here: [https://www.youtube.com/watch?v=nDJETVboL4Y](https://www.youtube.com/watch?v=nDJETVboL4Y)
* Text tutorial here: [https://www.raspberrypi.com/documentation/computers/linux_kernel.html](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#contribute-to-the-linux-kernel)

SSH to a Raspberry Pi.
```bash
ssh username@hostname.local
```

Update and Upgrade all packages.
```bash
sudo apt update
sudo apt upgrade -y
```

Install GIT and clone the repository.
```bash
sudo apt install git
git clone --depth=1 https://github.com/raspberrypi/linux
```

Install dependencies needed to build kernel.
```bash
sudo apt install bc bison flex libssl-dev make
```

### 2. Configure Build
For a Raspberry Pi 4b and 64bit OS, set the following variables. For different models, refer to the [tutorial](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#contribute-to-the-linux-kernel).
```bash
cd linux
KERNEL=kernel8
make bcm2711_defconfig
```

Set a custom name of your new kernel.
```bash
CONFIG_LOCALVERSION="-v7l-MY_CUSTOM_KERNEL"
```

Use the CLI configurator tool to set the kernel to RT. Install the package needed for the CLI and run.
```bash
sudo apt install libncurses-dev
make menuconfig
```

Inside the tool do the following.
* Press `\` to search and enter `PREEMPT_RT`.
* Press `1` to select the first search result.
* Using arrow keys, select `Fully Preemptible Kernel (Real-Time)`.
* Select `save`, press `enter`, and then `exit`. 

The changes can be found in the `~/linux/config` file as `CONFIG_PREEMPT_RT=y`.

### 3. Build
This step takes up to 3 hours. During the build, do not disconnect from the SSH session, or else the variables will not be saved.

Run the build.
```bash
make -j6 Image.gz modules dtbs
```

### 4. Install Kernel
Install kernel modules.
```bash
sudo make -j6 modules_install
```

Create a backup image of the current kernel, install the fresh kernel image, overlays, README, and unmount the partitions.
```bash
sudo cp /boot/firmware/$KERNEL.img /boot/firmware/$KERNEL-backup.img
sudo cp arch/arm64/boot/Image.gz /boot/firmware/$KERNEL.img
sudo cp arch/arm64/boot/dts/broadcom/*.dtb /boot/firmware/
sudo cp arch/arm64/boot/dts/overlays/*.dtb* /boot/firmware/overlays/
sudo cp arch/arm64/boot/dts/overlays/README /boot/firmware/overlays/
```

### 5. Reboot
Reboot the Raspberry Pi to load the new Kernel. The old kernel is backed up as `$KERNEL-backup.img`.

---

## EtherCAT Master
The goal is to use SOEM first, in combination with PySOEM to create simple apps to communicate with the motor.

### 1. SOEM
Install all necessary dependencies.
```bash
sudo apt install git cmake build-essential -y
```

Clone the SOEM repository.
```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git
```

Create a build directory, configure and compile the code.
```bash
cd SOEM
mkdir build
cd build
cmake ..
make
```

Now you can test scanning slaves using:
```bash
sudo ./samples/slaveinfo/slaveinfo eth0
```

### 2. PySOEM
Create venv inside a designated directory and install Python SOEM library.
```bash
# venv
python -m venv .venv
source .venv/bin/activate

# library
pip install pysoem
```

Try to run a test script to see if PySOEM works.
```python
import pysoem
import sys

NETWORK_IFACE = 'eth0' 

def test_connection():
    master = pysoem.Master()
    
    try:
        master.open(NETWORK_IFACE)
        print(f"Successfully opened {NETWORK_IFACE}")
    except Exception as e:
        print(f"Failed to open interface: {e}")
        sys.exit(1)

    # config_init() scans the network for slaves
    if master.config_init() > 0:
        print(f"\nSuccess! Found {len(master.slaves)} EtherCAT slave(s):")
        for i, slave in enumerate(master.slaves):
            print(f"  [{i+1}] {slave.name}")
    else:
        print("\nNo slaves found. Check your ethernet cable and ensure the Maxon drive is powered on.")

    master.close()

if __name__ == '__main__':
    test_connection()
```

To run the Python file as root but with the venv active, use this command:
```bash
sudo ~/PySOEM/.venv/bin/python ~/PySOEM/test.py
```

This enables the ethernet access to Python script and can use the EtherCAT.

### 3. PySOEM testing scripts
To test if the PySOEM can read data from HEJ, run this script.

#### Switch to Safe-Op state

```python
import time
import struct
import pysoem

def run_ethercat_test():
    # initialize master
    master = pysoem.Master()
    interface = 'eth0'
    
    print(f"Opening network interface: {interface}...")
    master.open(interface)
    
    # scan network for slaves
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found! Make sure the drive is powered on and connected.")
        master.close()
        return

    print(f"Success! Found {len(master.slaves)} slave(s).")
    device = master.slaves[0]
    print(f"Slave Name: {device.name}")
    
    # read SDO
    try:
        vendor_id = struct.unpack('<I', device.sdo_read(0x1018, 1))[0]
        product_code = struct.unpack('<I', device.sdo_read(0x1018, 2))[0]
        print(f"  -> Vendor ID: {hex(vendor_id)}")
        print(f"  -> Product Code: {hex(product_code)}")
    except Exception as e:
        print(f"Could not read SDO info: {e}")

    # map PDO
    master.config_map()

    # move to SAFE-OP state
    print("Transitioning to Safe-Operational state...")
    if master.state_check(pysoem.SAFEOP_STATE, timeout=50000) != pysoem.SAFEOP_STATE:
        print("Failed to reach Safe-Operational State.")
        master.close()
        return
        
    print("Drive is now in SAFE-OPERATIONAL mode. Reading 5 seconds of live data...")
    print("-" * 60)
    print(f"{'Iteration':<10} | {'Statusword':<12} | {'Actual Position':<15}")
    print("-" * 60)

    try:
        for i in range(50):
            # request process data
            master.receive_processdata()
            
            # unpack raw bytes
            # 2 bytes (H = unsigned short) for STATUSWORD, 4 bytes (i = int) for POSITION
            if device.input:
                print(f"Iteration {i+1}: Received {len(device.input)} bytes of data from the HEJ.")
                print(f"Raw hex: {device.input.hex()}")
            else:
                print(f"Iteration {i+1:<9}: Input buffer is empty.")

            # send empty frame to keep connection alive
            master.send_processdata()
            
            time.sleep(0.1)
            
    finally:
        # shutdown
        print("-" * 60)
        print("Reverting drive back to Init State and closing connection...")
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Network closed successfully.")

if __name__ == "__main__":
    run_ethercat_test()
```

#### Check the mode of operation
```python
import struct
import pysoem

def check_active_mode():
    master = pysoem.Master()
    interface = 'eth0'
    
    master.open(interface)
    
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found!")
        master.close()
        return

    device = master.slaves[0]
    
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout=50000)

    print("\nReading Modes of Operation Display (Object 0x6061)...")
    try:
        # Object 0x6061, Sub-index 0 is a 1-byte signed integer (SINT)
        raw_mode = device.sdo_read(0x6061, 0)
        mode = struct.unpack('<b', raw_mode)[0]
        
        # specific modes for this HEJ
        mode_names = {
            -60:    "[-60] Cogging Compensation Position Offset Identification Mode (CLOI)",
            -62:    "[-62] Joint Freeze Mode (JF)",
            -64:    "[-64] [DEFAULT] Joint Position Velocity Torque Mode (JPVT)",
            1:      "[1] [FORBIDDEN] Profile Position Mode (PPM)",
            3:      "[3] Profile Velocity Mode (PVM)",
            6:      "[6] Homing Mode",
            8:      "[8] Cyclic Synchronous Position Mode (CSP)",
            9:      "[9] Cyclic Synchronous Velocity Mode (CSV)",
            10:     "[10] Cyclic Synchronous Torque Mode (CST)",
            11:     "[11] Cyclic Synchronous Torque Mode with Commutation Angle (CSTCA) - internal test only"
        }
        
        active_mode_name = mode_names.get(mode, f"Custom/Unknown Mode (Raw Value: {mode})")
        
        print(f"> SUCCESS! Current mode of operation:")
        print(active_mode_name)
        
    except Exception as e:
        print(f"Could not read mode object: {e}")
        print("Tip: If it fails, ensure no other script is running and the drive is powered.")
        
    finally:
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()

if __name__ == "__main__":
    check_active_mode()
```

#### Set the mode of operation to CSV and check state
```python
import time
import struct
import pysoem

# this script sets the OP mode to 9 (CSV) and then reads the OP mode from the HEJ if its set correctly

def set_and_verify_mode():
    master = pysoem.Master()
    interface = 'eth0'
    
    print(f"Opening network interface: {interface}...")
    master.open(interface)
    
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found!")
        master.close()
        return

    device = master.slaves[0]
    print(f"Connected to: {device.name}")
    
    # Force the drive to PRE-OP state to unlock configuration mailboxes
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout=50000)

    print("\n[STEP 1] Writing CSV Mode (9) to Modes of Operation (0x6060)...")
    try:
        # 0x6060 is a 1-byte signed integer (b)
        device.sdo_write(0x6060, 0, struct.pack('<b', 9))
        print(" -> SDO Write sent successfully.")
        
        # Give the drive's microchip a brief 100ms window to process the state change
        time.sleep(0.1) 
        
    except Exception as e:
        print(f" -> SDO Write Failed: {e}")
        master.close()
        return

    print("\n[STEP 2] Verifying change via Modes of Operation Display (0x6061)...")
    try:
        # Read back from 0x6061 (1-byte signed integer)
        raw_mode = device.sdo_read(0x6061, 0)
        current_mode = struct.unpack('<b', raw_mode)[0]
        
        print("-" * 50)
        if current_mode == 9:
            print(f"SUCCESS! The drive confirms it has shifted to mode: {current_mode} (CSV)")
        else:
            print(f"NOTICE: The drive returned mode: {current_mode}")
            print("It might require PDO mappings to be loaded before it fully switches displays.")
        print("-" * 50)
        
    except Exception as e:
        print(f" -> SDO Read Failed: {e}")
        
    finally:
        # Gracefully drop the network state and close the port
        print("\nReverting network to safe state and closing...")
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Done.")

if __name__ == "__main__":
    set_and_verify_mode()
```

#### Switch to Operational mode and spin motor (not spinning yet)
```python
import time
import struct
import pysoem

def run_ethercat_csv_harden():
    master = pysoem.Master()
    interface = 'eth0'
    
    print(f"Opening network interface: {interface}...")
    master.open(interface)
    
    if master.config_init() <= 0:
        print("Error: No EtherCAT slaves found!")
        master.close()
        return

    device = master.slaves[0]
    print(f"Connected to: {device.name}")
    
    # Move to PRE-OP
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, timeout=50000)

    print("Configuring Mode (9) and Velocity Mapping...")
    try:
        device.sdo_write(0x6060, 0, struct.pack('<b', 9))  # CSV Mode
        time.sleep(0.05)
        device.sdo_write(0x1C12, 0, struct.pack('<B', 0))
        device.sdo_write(0x1C13, 0, struct.pack('<B', 0))
        time.sleep(0.05)
        device.sdo_write(0x1C12, 1, struct.pack('<H', 0x1603)) 
        device.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A03)) 
        time.sleep(0.05)
        device.sdo_write(0x1C12, 0, struct.pack('<B', 1))
        device.sdo_write(0x1C13, 0, struct.pack('<B', 1))
    except Exception as e:
        print(f"SDO Setup failed: {e}")
        master.close()
        return

    master.config_map()
    
    # Transition to SAFE-OP
    print("Transitioning to SAFE-OP...")
    master.state = pysoem.SAFEOP_STATE
    master.write_state()
    master.state_check(pysoem.SAFEOP_STATE, timeout=50000)

    print("Transitioning to OP state while feeding the watchdog...")
    master.state = pysoem.OP_STATE
    master.write_state()
    
    # Loop for 2 seconds, continuously sending frames to trick the watchdog
    # and push the state machine into Operational mode
    for _ in range(100):
        master.receive_processdata()
        
        # 0x0080 is the standard CiA 402 command to "Reset Fault" 
        # Sending this clears any latent errors blocking the OP transition
        device.output = struct.pack('<Hi', 0x0080, 0) 
        
        master.send_processdata()
        time.sleep(0.01)
        
        if master.state == pysoem.OP_STATE:
            break

    if master.state_check(pysoem.OP_STATE, timeout=10000) != pysoem.OP_STATE:
        print(f"Stuck! Current actual state code is: {master.state}")
        print("Check if your main motor DC power supply is actually switched on.")
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        return
        
    print("\n>>> SUCCESS! DRIVE IS LIVE IN OP MODE. RUNNING CRAWL... <<<")
    print("-" * 60)

    try:
        boot_sequence = [0x0006, 0x000F]
        target_rpm = 150             

        for i in range(300): 
            master.receive_processdata()
            
            if device.input and len(device.input) >= 6:
                status_word, act_position = struct.unpack('<Hi', device.input[:6])
            else:
                status_word, act_position = 0, 0

            if i < 40:
                control_word = boot_sequence[0] # 0x0006
                commanded_speed = 0
            else:
                control_word = boot_sequence[1] # 0x000F
                commanded_speed = target_rpm

            device.output = struct.pack('<Hi', control_word, commanded_speed)
            master.send_processdata()
            
            if i % 30 == 0:
                print(f"Status: {hex(status_word)} | Encoder Position: {act_position}")
                
            time.sleep(0.01)
            
    finally:
        print("\nSafely shutting down...")
        if device.output:
            device.output = struct.pack('<Hi', 0x0000, 0)
            master.send_processdata()
            master.receive_processdata()
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Closed.")

if __name__ == "__main__":
    run_ethercat_csv_harden()
```