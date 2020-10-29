# Hey Emacs, this is a -*- makefile -*-
##############################################################################
# Compiler settings
#
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
AR = arm-none-eabi-ar
NM = arm-none-eabi-nm
HEX = $(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse -R .lock -R .signature
EEP = $(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT)
BIN = $(OBJECOPY) -O binary

COMMON_VPATH += $(LIB_PATH)/atsam/DFP/$(MCU_FAMILY)/include
COMMON_VPATH += $(LIB_PATH)/atsam/CMSIS/5.5.1/include

COMPILEFLAGS += -funsigned-char
COMPILEFLAGS += -funsigned-bitfields
COMPILEFLAGS += -ffunction-sections
COMPILEFLAGS += -fshort-enums
COMPILEFLAGS += -fno-inline-small-functions
COMPILEFLAGS += -fno-strict-aliasing
COMPILEFLAGS += -mfloat-abi=hard
COMPILEFLAGS += -mfpu=fpv4-sp-d16
COMPILEFLAGS += -mthumb

#ALLOW_WARNINGS = yes

CFLAGS += $(COMPILEFLAGS)

CXXFLAGS += $(COMPILEFLAGS)
CXXFLAGS += -fno-exceptions -std=c++11

LDFLAGS +=-Wl,--gc-sections
LDFLAGS += -Wl,-Map="%OUT%%PROJ_NAME%.map"
LDFLAGS += -Wl,--start-group
LDFLAGS += -Wl,--end-group
LDFLAGS += --specs=rdimon.specs
LDFLAGS += -T$(LIB_PATH)/atsam/DFP/samd21a/gcc/gcc/samd21g18a_flash.ld

OPT_DEFS += -DPROTOCOL_ATSAM

MCUFLAGS = -mcpu=$(MCU)
MCUFLAGS += -D__$(ATSAM)__

# List any extra directories to look for libraries here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRALIBDIRS =

bin: $(BUILD_DIR)/$(TARGET).bin cpfirmware sizeafter
	$(COPY) $(BUILD_DIR)/$(TARGET).bin $(TARGET).bin;

flash: $(BUILD_DIR)/$(TARGET).bin cpfirmware sizeafter
	$(PRINT_OK); $(SILENT) || printf "$(MSG_FLASH_BOOTLOADER)"
