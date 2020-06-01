# pulsar_hardware

This package contains code for the physical robot hardware, to be compiled and loaded to the ESP-32.

Due to restriction placed upon the project by COVID-19 the robots were not able to be fully tested, and additional simulation work was required instead. As such, this package is unfinished, but has been included for completion.

## Building

This package builds a little weirldy - it should be built when `$ catkin_make` is called from the root of the workspace, like every other package. However,it uses a different compiler toolchain, so the CMakeLists within will run a script to do the actual compiling via `build_esp.sh`. This shouldn't be called by hand.

If the package isn't compiling for some reason, you can force a recompile by modifying any CMakeList.txt file in any of the packages (e.g. via `$touch CmakeLists.txt`), as the building actually happens during the searching phase rather than the building phase.

## Classes

### I2C

A class to represent an I2C interface with an arbitrary component. Implements reading and writing and a scan function. See header file for documentation.

### Led

A class to represent a simple togglable LED. Can be assigned on or off via the = operator, or via the toggle function. See header file for documentation.

### MotorDriver

A class to drive a motor PWM duty cycle. Contains functions to stop and update the duty cycle. See header file for documentation.
