# BloodPressure
# Setup Instructions
What you need to download:
1. GNU Arm Embedded Toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
  Update the Makefile.posix file with the location and version of this toolchain. This file is located in sdk/components/toolchain/gcc
  Here is an example of what a correct Makefile.posix would look like:
  GNU_INSTALL_ROOT := /home/kishbrao/Documents/gcc-arm-none-eabi-6-2017-q2-update/bin/
  GNU_VERSION := 6.3.1
  GNU_PREFIX := arm-none-eabi
 
2. Nordic CLI: https://www.nordicsemi.com/eng/nordic/Products/nRF51822/nRF5x-Command-Line-Tools-Linux64/51386
This is needed for use of nrfjprog and mergehex. When you install the two binaries you need to add nrfjprog to your path. Add it to your path in your ~/.bashrc as follows:
export PATH=$PATH:/home/kishbrao/Documents/eecs473/nordicTools/nrfjprog

3. Segger J-Link Tools: https://www.segger.com/downloads/jlink
Needed for using J-links


Setup Structure:
Include/ : Contains our own custom headers
lib/ : Contains our own .c files. This includes main and our other libraries
sdk/ : All relevant components of the sdk have been stripped and placed here 
linker/ : Contains the linker script that properly sets up the memory regions of the device
.gitignore : Please add build/ to your gitignore

Makefile usage
nrfjprog -e: Erase all memory (non-voltaile and volatile) on device
make flash_softdevice : Uploads the softdevice to the device
make flash: Uploads application to the device

Note that you only need to do make flash most times as the softdevice is already on all of our devboards. The full instructions are just here in case things get messed up.


Using Segger RTT:
1. In one terminal type JLinkexe
2. Use all default settings except type SWD (For serial wire debugging)
3. In another terminal type JLinkRTTClient
4. In your code use NRF_LOG_INFO("Hello World"); 
5. Ensure that the proper libraries are included 
6. In sdk/config/sdk_config.h make sure that NRF_LOG_BACKEND_RTT_ENABLED 1 is there









