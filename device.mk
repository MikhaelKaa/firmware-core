#  stm32f407vg device config

HEAP_SIZE  ?= 0x0400
STACK_SIZE ?= 0x0800

# C sources (kernel)
C_SOURCES += core/kernel/kernel.c
C_SOURCES += core/kernel/syscalls.c
C_SOURCES += core/kernel/sysmem.c
C_SOURCES += core/kernel/svccalls.c
C_SOURCES += core/kernel/drv_face.c

# C sources (drivers)
C_SOURCES += core/drivers/rcc/rcc.c
C_SOURCES += core/drivers/rng/rng.c
C_SOURCES += core/drivers/uart/uart1.c
C_SOURCES += core/drivers/uart/uart2.c
C_SOURCES += core/drivers/mem/mem.c
C_SOURCES += core/drivers/rtc/rtc.c
C_SOURCES += core/drivers/rtc/rtc_time.c
C_SOURCES += core/drivers/led/pwm_led.c
C_SOURCES += core/drivers/w25q/w25q.c
C_SOURCES += core/drivers/usb_cdc/usb_cdc.c
C_SOURCES += core/drivers/usb_cdc/usb_desc.c
C_SOURCES += core/drivers/adc/adc.c

# C includes
C_INCLUDES += -Icore/lib/time
C_INCLUDES += -Icore/vendor/CMSIS
C_INCLUDES += -Icore/kernel
C_INCLUDES += -Icore/drivers/rcc
C_INCLUDES += -Icore/drivers/rng
C_INCLUDES += -Icore/drivers/uart
C_INCLUDES += -Icore/drivers/mem
C_INCLUDES += -Icore/drivers/rtc
C_INCLUDES += -Icore/drivers/led
C_INCLUDES += -Icore/drivers/w25q
C_INCLUDES += -Icore/drivers/usb_cdc
C_INCLUDES += -Icore/drivers/adc

# ASM sources
ASM_SOURCES +=  core/vendor/startup/startup_stm32f407xx.s
# AS includes
AS_INCLUDES += 

# AS defines
AS_DEFS += 

# C defines
C_DEFS += -DSTM32F407xx
C_DEFS += -DBAREMETAL
C_DEFS += -DSYSTEM_CORE_CLOCK=168000000U

# CFLAGS
# cpu
CPU = -mcpu=cortex-m4
# fpu
FPU = -mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI = -mfloat-abi=hard

# link script
LDSCRIPT = core/vendor/linker/STM32F407VGTx_FLASH.ld
LDFLAGS += -Wl,--defsym=_Min_Heap_Size=\$(HEAP_SIZE)
LDFLAGS += -Wl,--defsym=_Min_Stack_Size=\$(STACK_SIZE)

OPENOCD_MCU_CFG ?= core/vendor/openocd/stm32f4x.cfg
