# What I need to create?
This is the future plan for the next phase of the development of the JPVT controller

## UniTree Startup Sequence
I need to recreate the startup sequence of the unitree robot, that works like this:

1. On startup, motors are disabled and unlocked.
2. when enabling the **developer mode**, motors are powered and they move to the zero position (probably zero for the encoders).
3. After that, the **dumping mode** is activated. This mode does not execute position control, but controls the velocity to zero.
4. In this phase, the motor is ready to accept commands - subscribes to a topic.

## New HEJ Startup Sequence
This basically copies the **UniTree Startup Sequence**, but is described in more detail referencing the current controller.

First version will only listen to commands from the UNIX local socket. Later, the DDS or the UniTree SDK will be implemented.

### 1. Phase: INIT
1. Script execution is called.

2. Program subsribes to the topic (listen for commands) right away so it know when to proceed to another phase.

3. It runs all the configuration commands and all the initialization with the **power stage off**.

### 2. Phase: DEVELOPER MODE
1. After receiving a command, it proceeds to the **developer mode**. In this stage, the motor gets **enabled and powered on**.

2. The PID gains are set to 0 and the motor is enabled. This ensures no movement, but the *filtered* and *fused* position values are calculated.

3. Maximal velocity and torque are set low. Target position is set to 0. PID gain values are set to non-zero, but low, to ensure the homing position is reached safely.

4. Position is reached with a certain tolerance. Now it waits for another command to proceed to another phase.

### 3. Phase: DAMPING MODE
1. Target velocity is set to zero with non-zero D gain, while PI gains are set to zero. This only damps the movement to a zero velocity, but can be overcome by hand - when released, it stays at the current position.

2. Now the HEJ is ready to accept movement commands.

## Communication over DDS
Next step will be implementing the DDS communication protocol which replaces the UNIX Socket messaging.