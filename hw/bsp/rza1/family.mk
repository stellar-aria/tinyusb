DEPS_SUBMODULES += hw/mcu/renesas/rza1

RZ_BSP = hw/mcu/renesas/rza1/rz_bsp
include $(TOP)/$(BOARD_PATH)/board.mk

CFLAGS += \
  -mcpu=cortex-a9 \
  -mfpu=neon \
  -mfloat-abi=hard \
  -DCFG_TUSB_MCU=OPT_MCU_RZA1X \
	-flto \
	-nostdlib \
	-nostartfiles \
	-ffreestanding

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

INC += \
	$(TOP)/$(RZ_BSP)/src/renesas/compiler/inc \
	$(TOP)/$(RZ_BSP)/src/renesas/application/inc \
	$(TOP)/$(RZ_BSP)/src/renesas/application/system \
	$(TOP)/$(RZ_BSP)/src/renesas/application/system/iodefines \

# # For freeRTOS port source
# FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/RX600

