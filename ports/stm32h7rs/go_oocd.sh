#!/bin/sh
# start up openocd for debug
export OPENOCD_SCRIPTS=/home/ericb/build/openocd/stm32oocd/OpenOCD/tcl/
~/build/openocd/stm32oocd/OpenOCD/src/openocd -f openocd_stlink.cfg 
