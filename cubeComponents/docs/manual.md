# CubeComponents Interface Drivers
The CubeComponents interface drivers defines the telemetry and telecommand interface.  The drivers purpose is to simplify the interfacing of a master with the specific slave component.

## Content
1. The *inc* folder contains the necessary structure and enumeration defines as well as function definitions for each transaction.
2. The *i2c* folder contains the source files for functions defines in its corresponding header file.  Each function creates the i2c packet and unpack the telemetry feedback.

## Installation
It is required to take the cubelib.h file along with the corresponding header and source file of the specific CubeComponent and make it part of the application's source.

There are a few functions that need to be implemented which the drivers require.  These functions are:
1. BSP_I2C_masterTransmit
2. BSP_TIME_Delay
