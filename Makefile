##########################
CROSSTOOL  = avr-
#list all source files [*.c]
SRCS       = main.c 
SRCS      += printf_port.c
#the last binary file
BIN        = arduino_nrf2401_bootloader
#MCU name
MCU        = atmega328p
# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the 
#     processor frequency. You can then use this symbol in your source code to 
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
F_CPU      = 16000000
# set log level :
#     0 - log disable to save flash size
#     1 - normal mode, only debug log disable: LOG(LOG_LEVEL_DEBUG, " %s %X ", ...)
#     2 - full mode, all log api enable 
LOG_LEVEL  = 0
#Boot Flash section size = 1024 words = 2048 bytes
#boot start address (word) = 0x3C00
#address = 0x3C00*2 = 0x7800
BOOTLOADER_ADDRESS = 0x7800
#
VPATH      = log
#
ifeq ($(shell uname -s), Linux)
	DEVICE_PORT = /dev/ttyUSB0  #linux
else
	DEVICE_PORT = COM10          #windows 
endif
#
CC         = $(CROSSTOOL)gcc
OBJDUMP    = $(CROSSTOOL)objdump
OBJCOPY    = $(CROSSTOOL)objcopy
SIZE       = $(CROSSTOOL)size
##########################
#saving  path for *.o *.d
TMPPATH    =  tmp
#saving  path for binary file
BINPATH    =  bin
OBJS       = $(SRCS:%.c=$(TMPPATH)/%.o)
TARGET     = $(BINPATH)/$(BIN)
##########################
MF_ABS_PATH = $(shell pwd)
#
CFLAG_INC   = $(patsubst %,-I%,$(VPATH))
# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL -D__BUILD_TIME__=\"$(shell date "+%Y-%m-%d\ %H:%M:%S")\"
#编译配置项
CFLAGS     = -Wall -Os -std=gnu99 -fshort-enums -g -nostartfiles
ifdef MCU
	CFLAGS += -mmcu=$(MCU)
	CFLAGS += $(CDEFS)
endif
CFLAGS    += $(CFLAG_INC)
ifdef LOG_LEVEL
	CFLAGS += -DLOG_LEVEL_SET=$(LOG_LEVEL)
endif
##########################
#链接配置项
LFLAGS     = -Wl,-Map=$(TARGET).map,--cref -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS)

all: dirs info $(TARGET)
#
dirs:
	@mkdir -p $(BINPATH) $(TMPPATH)
#
info:
	@echo CFLAGS = $(CFLAGS)
	@echo LFLAGS = $(LFLAGS)
	@echo ================================================
#
$(TARGET): % :%.elf
	@echo ================================================
	@#ELF info
	$(OBJDUMP) -x -d $< > $@.dump
	@#
	$(OBJDUMP) -S -I -z $< > $@.s
	@#output binary image
	$(OBJCOPY) -j .text -j .data -O binary $< $@
	$(OBJCOPY) -j .text -j .data -O ihex $< $@.hex
	@#show memory usage
	@echo ================================================
	@$(SIZE) $<
	@echo ================================================
#link
%.elf: $(OBJS)
	@echo Linking [$@] $^
	@$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^
#compile
$(TMPPATH)/%.o: %.c
	@echo Compiling [$<]
	@$(CC) -c $(CFLAGS) -MMD -MP -MF $(@:.o=.d) -o $@ $<
	@$(OBJDUMP) -S $@ > $(@:.o=.s)
upload:
	avrdude -c usbasp -p$(MCU) -U lfuse:w:0xff:m -U hfuse:w:0xda:m -U efuse:w:0xfd:m
	avrdude -c usbasp -p$(MCU) -e -U flash:w:$(TARGET).hex:i
clean:
	-rm -f -R $(BINPATH)
	-rm -f -R $(TMPPATH)

#include dependency files
-include $(SRCS:%.c=$(TMPPATH)/%.d)
.PHONY: all clean upload dirs