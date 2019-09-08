
# Put your stlink folder here so make burn will work.
STLINK=st-flash

# Put your source files here (or *.c, etc)
SRCS=main.c system_stm32f4xx.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=test

# Put your STM32F4 library code directory here
STM_COMMON=./STM32F4-Discovery_FW_V1.1.0

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.

vpath %.c \
$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src \
$(STM_COMMON)/Libraries/STM32_USB_OTG_Driver/src \
$(STM_COMMON)/Libraries/STM32_USB_Device_Library/Core/src \
$(STM_COMMON)/Libraries/STM32_USB_Device_Library/Class/cdc/src

# Includes
CFLAGS += -I.
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -I$(STM_COMMON)/Libraries/STM32_USB_OTG_Driver/inc
CFLAGS += -I$(STM_COMMON)/Libraries/STM32_USB_Device_Library/Core/inc
CFLAGS += -I$(STM_COMMON)/Libraries/STM32_USB_Device_Library/Class/cdc/inc

# add startup file to build
SRCS += $(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s
OBJS = $(SRCS:.c=.o)

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@;
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex;
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32F4
burn: proj
	$(STLINK) write $(PROJ_NAME).bin 0x8000000
