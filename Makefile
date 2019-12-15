
# Put your stlink folder here so make burn will work.
STLINK=st-flash

# Put your source files here (or *.c, etc)
SRCS=main.c system_stm32f4xx.c stm32f4_gpio.c stm32f4_usart.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=test

# Put your STM32F4 library code directory here
STM_COMMON=.

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.

#vpath %.c \
#$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src \
#$(STM_COMMON)/Libraries/STM32_USB_OTG_Driver/src \
#$(STM_COMMON)/Libraries/STM32_USB_Device_Library/Core/src \
#$(STM_COMMON)/Libraries/STM32_USB_Device_Library/Class/cdc/src

# Includes
#CFLAGS += -I.
#CFLAGS += -D USE_STDPERIPH_DRIVER
CFLAGS += -I$(STM_COMMON)/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/CMSIS/Include

# add startup file to build
SRCS += $(STM_COMMON)/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s
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
