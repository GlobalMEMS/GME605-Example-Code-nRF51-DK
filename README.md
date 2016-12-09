nRF51-DK + GME605 Orientation Demo
==================================

Requirements
-----------
- nRF51-DK
- nRF51 SDK: tested with Version 10.0.0
- GCC ARM embedded compiler: tested with gcc-arm-none-eabi-4_9-2015q1-20150306
- GME605 or (GMA303 + GMC303)

I2C Connections
---------------
- Use I2C0
  - SCL: P0.07
  - SDA: P0.30
- Accelerometer (GMA303) I2C 7-bit slave address: 0x18
- Magnetometer (GMC303)  I2C 7-bit slave address: 0x0C

Makefile
--------
- Modify the location of the SDK to your actual directory
```
#
# The base directory for nRF51 SDK
#
nRF51_SDK_ROOT := /C/Data/nRF51_SDK/V10_0_0
```

- Modify the location of the GNU tools to your actual directory
```
GNU_INSTALL_ROOT := C:/Program Files (x86)/GNU Tools ARM Embedded/4.9 2015q1
```

- Execute `make all` to build and flash the program

- Exectute `make help` for available targets

Sensor Layout Pattern
---------------------
Sensor layout pattern is defined by the following macro in the "main.c"
```
#define MAG_LAYOUT_PATTERN          PAT1                 //magnetometer layout pattern
#define ACC_LAYOUT_PATTERN          PAT6                 //accelerometer layout pattern
```

Please refer to the "Sensor_Layout_Pattern_Definition.pdf" document for the definition and modify accordingly to fit your actual layout.

Usage of AutoNil
----------------
 * The program will do an offset AutoNil when executed. Hold the g-sensor steady and maintain in level, then press 'y' after the program prompt for input.
 * You may change the `DATA_AVE_NUM` macro in the gSensor_autoNil.h for the moving averae order for the offset estimation. Defautl is 32.

Debug print
-----------
To enable debug output, uncomment the following macro in "akmdfs/AKFS_Log.h" and rebuild the project.
```
//#define ENABLE_AKMDEBUG	1
```

AKMDFS Library
--------------
The AKMDFS demo library is adapted from [Compass Control Program for Android Open Source Project](https://github.com/akm-multisensor/AKMDFS).
   
