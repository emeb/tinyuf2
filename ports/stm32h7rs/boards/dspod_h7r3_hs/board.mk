QSPI_FLASH = GD25Q32C_QSPI

CFLAGS += \
  -DSTM32H7R3xx \
  -DHSE_VALUE=12000000U \
  -DTINYUF2_LED=1 \
  -DBOARD_QSPI_FLASH_EN=1 \
  -DBOARD_AXISRAM_EN=1 \
  -D$(QSPI_FLASH)\
  -DBOARD_FLASH_APP_START=0x90000000 \
  -DTRAP_EXC \
  -DBOARD_HIGHSPEED=1 \

#
# Need to tweak tinyusb/src/common/tusb_mcu.h line 323 with this:
#  #define TUP_RHPORT_HIGHSPEED    BOARD_HIGHSPEED
# to enable switching full/high speed

MCU = h7r3xx

LOG = 0

SRC_S += \
  $(ST_CMSIS)/Source/Templates/gcc/startup_stm32h7r3xx.s
