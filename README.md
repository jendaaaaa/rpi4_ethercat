# EtherCAT on Raspberry Pi 4b

## Before Installing

### 1. Raspberry Pi Imager
* Set up image of Ubuntu Server 24.04.4 LTS (64-bit).
* Configure WiFi name and password.
* Enable SSH (i used login and password).

---

### 2. Ubuntu Server
* Plug SSD into Raspberry Pi 4b.
* Turn it on.
* Wait for about 5 mins to go through initial setup.

## Installation

### 3. Enahle SSH
This step was unsuccessful, RPi can `ping google.com` but cannot ping my PC, also PC cannot ping RPi. If you want to set up the rest remotely, enable SSH.

To get SSH status, run:
```bash
sudo systemclt status ssh
```
SSH is probably **Loaded** but **Inactive**. To activate, run:

```bash
sudo systemctl enable --now ssh
```
For me it didn't work, my RPi is still unreachable as well as my PC from RPi.

---

### 4. Enable RT
Update and Upgrade all packages.
```bash
sudo apt update
sudo apt upgrade -y
```

Enable Ubuntu Pro (register at [ubuntu.com/pro](ubuntu.com/pro) and get the **Free Personal Token**).

```bash
sudo pro attach xxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```
Enable RealTime Kernel for Raspi.
```bash
sudo pro enable realtime-kernel --variant=raspi
```
Accept all `[Y]`.

Reboot RPi.
```bash
sudo reboot
```

---

### 5. Test the RT kernel
Verify the `PREEMPT_RT` is active by running:
```bash
uname -a
```

My output was:
```
Linux [USERNAME] 6.8.0-2038-raspi-realtime #39-Ubuntu SMP PREEMPT_RT [DATE TIME] aarch64 aarch64 aarch64 GNU/Linux
```

Once you see the RT kernel, you can move on.

---

### 6. Libraries
Install all necessary dependencies.
```bash
sudo apt install git cmake build-essential -y
```

---

### 7. SOEM (Simple Open EtherCAT Master)
Create a directory for all new files (optional).

```bash
mkdir docs
cd docs
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

Finally install SOEM from `SOEM/build` - copy headers to `/usr/local/include/soem` and the library to `/usr/local/lib`.
```bash
sudo make install
sudo ldconfig
```

## Create a Project
### 8. Workspace
Create you workspace directory.
```bash
cd ~
mkdir my_ethercat_app
cd my_ethercat_app
```

Create the build system.
```bash
nano CMakeLists.txt
```

Create a configuration.
```cmake
cmake_minimum_required(VERSION 3.10)
project(MyEtherCATApp)

# Use C99 standard
set(CMAKE_C_STANDARD 99)

# Find the SOEM library we just installed
find_path(SOEM_INCLUDE_DIR NAMES ethercat.h PATHS /usr/local/include/soem)
find_library(SOEM_LIBRARY NAMES soem PATHS /usr/local/lib)

# Include directories
include_directories(${SOEM_INCLUDE_DIR})

# Create the executable
add_executable(my_app main.c)

# Link SOEM and the Real-Time Threading library (pthread and rt)
target_link_libraries(my_app ${SOEM_LIBRARY} pthread rt)
```

Save and exit (`CTRL+o`, `Enter`, `CTRL+x`).

### Script

Create your main file with `nano main.c`.
```c
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include "ethercat.h"

// Your network interface name
char IOmap[4096];
char ifname[] = "end0"; 

void* ethercat_rt_loop(void *arg) {
    // 1. Tell the RT Kernel to give this thread MAXIMUM PRIORITY
    struct sched_param param;
    param.sched_priority = 98; // 99 is max, 98 is highly recommended for EtherCAT
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    printf("Starting Real-Time EtherCAT Loop...\n");

    // 2. The Cyclic Loop
    while (1) {
        // A. Send Process Data to slaves
        ec_send_processdata();

        // B. Wait for data to come back (receive)
        ec_receive_processdata(EC_TIMEOUTRET);

        // C. YOUR CONTROL LOGIC GOES HERE
        // e.g., Read sensor -> Calculate PID -> Write motor speed

        // D. Sleep for exactly 1 millisecond to maintain the cycle
        osal_usleep(1000); 
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    printf("Initializing SOEM on %s...\n", ifname);

    // Initialize the network port
    if (ec_init(ifname)) {
        printf("ec_init succeeded.\n");

        // Find and configure slaves
        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found.\n", ec_slavecount);

            // Map the IO variables
            ec_config_map(&IOmap);

            // Transition hardware to SAFE_OP, then OP state
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);

            // Spawn the Real-Time thread
            pthread_t rt_thread;
            pthread_create(&rt_thread, NULL, ethercat_rt_loop, NULL);

            // Keep main program alive
            while(1) {
                sleep(1); 
            }
        } else {
            printf("No slaves found!\n");
        }
    } else {
        printf("Failed to initialize interface!\n");
    }
    return 0;
}
```

### Compile and run
Build your program.
```bash
mkdir build
cd build
cmake ..
make
```

Run your program as root to get access to the Ethernet HW.
```bash
sudo ./my_app
```