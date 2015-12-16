#This is a special custom crazyflie firmware created and built by me

The changes are 
1. There are multiple options for controllers. Setting the nl_controller t0
	a. 0 - Normal PIC Controller
	b. 1 - Non Linear Controller described here - http://www.math.ucsd.edu/~mleok/pdf/LeLeMc2011_nrtc.pdf
	c. 2 - A BackStepping Controller described here - http://dar.aucegypt.edu/bitstream/handle/10526/3965/Heba_ElKholy_Thesis_S2014.pdf?sequence=1
Note - These new controllers have not been tuned properly, hence do not work

2. Initial Support for DWM1000 Chip. It uses the libraries distributed by the manufacturer. Can read registers and stuff. No protocol implemented yet. (Found in SPI1.c in Drivers and DWM .c in hal)

3. Support for Futaba SBUS. Needs an external invberter connected to the input pin (Found in uart1.c in Drivers). This can be used for controlling the crazyflie as well.
 
4. Supporting mods for sending position setpoints and current position for internal positon hold. 
	a. Port 0x06 - Current Position
	b. Port 0x07 - Destination Position
	c. Port 0x08 - Current State like position hold attitude mode etc.

5. The crazyflie will start positon mode if yawed aggresively.

6. Other things I might have forgotten



# Crazyflie 1.0/2.0 Firmware  [![Build Status](https://api.travis-ci.org/bitcraze/crazyflie-firmware.svg)](https://travis-ci.org/bitcraze/crazyflie-firmware)

This project contains the source code for the Crazyflie 1.0/2.0 firmware. 

## Dependencies

You'll need to use either the Crazyflie VM or install some of an ARM toolchain.

### OS X
```bash
brew tap PX4/homebrew-px4
brew install gcc-arm-none-eabi
```

### Debian/Ubuntu

> `TODO: Please share!`

### Arch Linux

```bash
sudo pacman -S community/arm-none-eabi-gcc community/arm-none-eabi-gdb community/arm-none-eabi-newlib
```

### Windows

> `TODO: Please share!`

## Compiling

### Crazyflie 1.0
Build with:
```
make PLATFORM=CF1
```

### Crazyflie 2.0
This is the dafault build so just running "make" is enough or:
```
make PLATFORM=CF2
```
### config.mk
To create custom build options create a file called config.mk in the root folder 
(same as Makefile) and fill it with options. E.g. 
```
PLATFORM=CF1
DEBUG=1
CLOAD=0
```
More information can be found on the 
[Bitcraze wiki](http://wiki.bitcraze.se/projects:crazyflie2:index)

## Folder description:
```
./              | Root, contains the Makefile
 + init         | Contains the main.c
 + config       | Configuration files
 + drivers      | Hardware driver layer
 |  + src       | Drivers source code
 |  + interface | Drivers header files. Interface to the HAL layer
 + hal          | Hardware abstaction layer
 |  + src       | HAL source code
 |  + interface | HAL header files. Interface with the other parts of the program
 + modules      | Firmware operating code and headers
 |  + src       | Firmware tasks source code and main.c
 |  + interface | Operating headers. Configure the firmware environement
 + utils        | Utils code. Implement utility block like the console.
 |  + src       | Utils source code
 |  + interface | Utils header files. Interface with the other parts of the program
 + platform     | Platform specific files. Not really used yet
 + tools        | Misc. scripts for LD, OpenOCD, make, version control, ...
 |              | *** The two following folders contains the unmodified files ***
 + lib          | Libraries
 |  + FreeRTOS  | Source FreeRTOS folder. Cleaned up from the useless files
 |  + STM32...  | Library folders of the ST STM32 peripheral libs
 |  + CMSIS     | Core abstraction layer
```
# Make targets:
```
all        : Shortcut for build
compile    : Compile cflie.hex. WARNING: Do NOT update version.c
build      : Update version.c and compile cflie.elf/hex
clean_o    : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean      : Clean every compiled files
mrproper   : Clean every compiled files and the classical editors backup files

cload      : If the crazyflie-clients-python is placed on the same directory level and 
             the Crazyradio/Crazyradio PA is inserted it will try to flash the firmware 
             using the wireless bootloader.
flash      : Flash .elf using OpenOCD
halt       : Halt the target using OpenOCD
reset      : Reset the target using OpenOCD
openocd    : Launch OpenOCD
```
