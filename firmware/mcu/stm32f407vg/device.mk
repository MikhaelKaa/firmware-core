#  stm32f407vg make

# C sources
C_SOURCES += mcu/$(MCU_PATH)/drivers/rcc/rcc.c
C_SOURCES += mcu/$(MCU_PATH)/drivers/rng/rng.c
C_SOURCES += mcu/$(MCU_PATH)/drivers/uart/uart1.c
C_SOURCES += mcu/$(MCU_PATH)/drivers/uart/uart2.c

# C includes
C_INCLUDES += -Imcu/$(MCU_PATH)/vendor/CMSIS
C_INCLUDES += -Imcu/$(MCU_PATH)/drivers/rcc
C_INCLUDES += -Imcu/$(MCU_PATH)/drivers/rng
C_INCLUDES += -Imcu/$(MCU_PATH)/drivers/uart

# ASM sources
ASM_SOURCES +=  ./mcu/$(MCU_PATH)/vendor/startup/startup_stm32f407xx.s
# AS includes
AS_INCLUDES += 

# AS defines
AS_DEFS += 

# C defines
C_DEFS += -DSTM32F407xx
C_DEFS += -DBAREMETAL


# CFLAGS
# cpu
CPU = -mcpu=cortex-m4
# fpu
FPU = -mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI = -mfloat-abi=hard

# link script
LDSCRIPT = ./mcu/$(MCU_PATH)/vendor/linker/STM32F407VGTx_FLASH.ld

OPENOCD_MCU_CFG ?= ./mcu/$(MCU_PATH)/vendor/openocd/stm32f4x.cfg