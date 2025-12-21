# root make

# common settings
# name
TARGET = firmware
# debug build
DEBUG = 1
# optimization
OPT = -Og
# Build path
BUILD_DIR = build

# default mcu path
MCU_PATH ?= stm32f407vg

# include mcu files
include ./mcu/$(MCU_PATH)/device.mk

sinclude ./local.mk

# C sources
C_SOURCES += kernel/kernel.c
C_SOURCES += kernel/syscalls.c
C_SOURCES += kernel/sysmem.c

# C includes
C_INCLUDES += -Ikernel


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU)
CFLAGS += $(C_DEFS)
CFLAGS += $(C_INCLUDES)
CFLAGS += $(OPT)
CFLAGS += -std=gnu11
CFLAGS += -Wall
CFLAGS += -Wextra
CFLAGS += -Wpedantic
CFLAGS += -Wconversion
# CFLAGS += -Wmissing-prototypes
# CFLAGS += -Wstrict-prototypes
CFLAGS += -fdata-sections
CFLAGS += -ffunction-sections
CFLAGS += -D_DEBUG
CFLAGS += -D_DEFAULT_SOURCE


ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################

# libraries
LIBS += -lc
LIBS += -lm
LIBDIR = 
LDFLAGS += $(MCU)
LDFLAGS += $(LIBS)
LDFLAGS += $(LIBDIR)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -specs=nano.specs
LDFLAGS += -specs=nosys.specs
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -u _printf_float
LDFLAGS += -u _scanf_float

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

#  default OpenOCD transport (see local.mk)
OPENOCD_TRANSPORT_CFG ?= ./firmware/mcu/common/openocd/stlink.cfg
# Load firmware.bin (hex) use OpenOCD
load:
	openocd -f $(OPENOCD_TRANSPORT_CFG) -f $(OPENOCD_MCU_CFG) -c init -c "reset halt" -c "flash write_image erase $(BUILD_DIR)/$(TARGET).hex" -c "reset run" -c "exit"

# *** EOF ***
