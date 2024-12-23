# Ethernet/IP Driver/Interface for controlling FANUC Robots in C++

This code emulates a PLC to directly write to registers of the FANUC robot controller and thus control position and speed of the end effector.

The robot needs to be running the attached TP Program to read out the registers.

**It is important to note that this way of controlling the robot does not yield optimal performance. It takes ~10ms to update or read registers, such as position setpoint or the "Movement done" register. Keep this in mind when using.**

**IMPORTANT: This library is not thread-safe and does not make any guarantees when functions are called from multiple threads! It is desgined to do a single thing on a single thread: Control FANUC robots. If you want to access data from other threads, you have to implement this by yourself, i.e. by having a central control thread that others can request data from.**

The repo uses a dev container so you only have to open it in VS Code and install the dev container extension pack. If you do not want to use VS Code, you can manually build and run the Dockerfile using the script `build-and-run.sh`. This will give you a tty inside the docker container and you can run `build-and-run.sh` again inside of it.