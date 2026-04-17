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