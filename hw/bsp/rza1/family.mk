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

SRC_C += \
	src/portable/renesas/rusb2/dcd_rusb2.c \
	src/portable/renesas/rusb2/hcd_rusb2.c \
	$(RZ_BSP)/src/renesas/application/system_Renesas_RZ_A1.c

INC += \
	$(TOP)/$(RZ_BSP)/src/renesas/compiler/inc \
	$(TOP)/$(RZ_BSP)/src/renesas/application/inc \
	$(TOP)/$(RZ_BSP)/src/renesas/application/system \
	$(TOP)/$(RZ_BSP)/src/renesas/application/system/inc \
	$(TOP)/$(RZ_BSP)/src/renesas/application/system/iodefines \

SRC_S += \
	$(RZ_BSP)/src/renesas/compiler/asm/start.S \
	$(RZ_BSP)/src/renesas/compiler/asm/ttb_init.S \
	$(RZ_BSP)/src/renesas/compiler/asm/reset_handler.S

LDFLAGS += -L$(TOP)/$(RZ_BSP)/src/renesas/compiler

# # For freeRTOS port source
# FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/RX600

