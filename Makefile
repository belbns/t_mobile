# Basic makefile K Sarkies
# Just add PROJECT = base name of main c file to compile
# Also add any other CFILES to be added in.

PROJECT		= t_mobile

# The libopencm3 library is assumed to exist in libopencm3/lib, otherwise add files here
CFILES		+= $(PROJECT).c periph.c jfes.c
CFILES		+= tasks.c list.c queue.c timers.c port.c event_groups.c heap_2.c

ENABLE_SEMIHOSTING	= 0

PREFIX		= arm-none-eabi
CC		= $(PREFIX)-gcc
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
GDB		= $(PREFIX)-gdb
NM 		= $(PREFIX)-nm
SIZE		= $(PREFIX)-size

DEBUG           ?= -ggdb3
CSTD            ?= -std=c99

LIBRARY_DIR	= ..
DRIVERS_DIR	= $(LIBRARY_DIR)/libopencm3
DRIVERS_SRC	= $(DRIVERS_DIR)/lib/stm32/f1
DRIVERS_INC	= $(DRIVERS_DIR)/include

FREERTOS_DIR	= $(LIBRARY_DIR)/FreeRTOS/FreeRTOSv10.2.1
FREERTOS_DEV	= $(FREERTOS_DIR)/Source/portable/GCC/ARM_CM3
FREERTOS_INC	= $(FREERTOS_DIR)/Source/include
FREERTOS_SRC	= $(FREERTOS_DIR)/Source
FREERTOS_MMG	= $(FREERTOS_DIR)/Source/portable/MemMang

LDSCRIPT	= $(DRIVERS_SRC)/stm32f103x8.ld

VPATH		+= $(DRIVERS_SRC)/ $(DRIVERS_SRC)/../
VPATH		+= $(FREERTOS_SRC)/ $(FREERTOS_DEV)/ $(FREERTOS_MMG)/

#CFLAGS		+= -Os -g -Wall -Wextra -I$(DRIVERS_INC)
CFLAGS		+= -Os $(CSTD) $(DEBUG) -Wall -Wextra -I$(DRIVERS_INC)
CFLAGS		+= -I$(FREERTOS_INC) -I$(FREERTOS_DEV) -I.
CFLAGS		+= -fno-common -mcpu=cortex-m3 -mthumb -msoft-float -MD -DSTM32F1
LDFLAGS		+= -I . -Wl,--start-group -lc -lgcc -Wl,--end-group
LDFLAGS		+= -T$(LDSCRIPT) -L$(DRIVERS_DIR)/lib -lopencm3_stm32f1
LDFLAGS		+= -nostartfiles -Wl,--gc-sections
LDFLAGS		+= -mthumb -march=armv7 -mfix-cortex-m3-ldrd -msoft-float

# semihosting
ifeq ($(ENABLE_SEMIHOSTING),1)
LDFLAGS		+= -lrdimon
CFLAGS		+= -DENABLE_SEMIHOSTING
endif

LDFLAGS		+= -lnosys

OBJS		= $(CFILES:.c=.o)

all: $(PROJECT).elf $(PROJECT).bin $(PROJECT).hex $(PROJECT).list $(PROJECT).sym
	$(SIZE) $(PROJECT).elf

$(PROJECT).elf: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex  $< $@

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary  $< $@

$(PROJECT).list: $(PROJECT).elf
	$(OBJDUMP) -S $< > $@

$(PROJECT).sym: $(PROJECT).elf
	$(NM) -n $< > $@

clean:
#	rm *.elf *.o *.d *.hex *.list *.sym *.bin
	@#printf "  CLEAN\n"
	$(Q)$(RM) *.elf *.o *.d *.hex *.list *.sym *.bin


# Using CC and CFLAGS will cause any object files to be built implicitely if
# they are missing. # We are searching an archive library opencm3_stm32f4.a
# which has been precompiled, so we don't need to recompile the DRIVERS source.
