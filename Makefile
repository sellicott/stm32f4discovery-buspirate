# Put your STM32F4 library code directory here
BUSPIRATE_DIR=buspirate
SRC_DIR=Src
INC_DIR=Inc
LIB_DIR=lib

STM_LIB_SRC = Drivers/STM32F4xx_HAL_Driver
STM_USB_CORE = Middlewares/ST/STM32_USB_Device_Library/Core
STM_USB_CDC = Middlewares/ST/STM32_USB_Device_Library/Class/CDC
CMSIS_CORE = Drivers/CMSIS/Include
CMSIS_STM32 = Drivers/CMSIS/Device/ST/STM32F4xx

TARGET=stm32f4xx

# Put your source files here (or *.cflashing a hex file on a stm32f4discovery, etc)

USR_SRC       := $(SRC_DIR)/*.c
#BUSPIRATE_SRC := $(BUSPIRATE_DIR)/*.c
STM_SRC       := $(STM_LIB_SRC)/Src/$(TARGET)_hal.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_spi.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_pcd.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_pcd_ex.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_gpio.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_exti.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_dma.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_dma_ex.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_dma2d.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_rcc.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_rcc_ex.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_flash.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_flash_ex.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_flash_ramfunc.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_pwr.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_pwr_ex.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_hal_cortex.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_rcc.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_usb.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_spi.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_pwr.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_dma.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_dma2d.c
STM_SRC       += $(STM_LIB_SRC)/Src/$(TARGET)_ll_gpio.c
#STM_SRC += $(CMSIS_STM32)/Source/Templates/system_$(TARGET).c
STM_SRC += $(STM_USB_CORE)/Src/*.c
STM_SRC += $(STM_USB_CDC)/Src/*.c
STARTUP := startup_stm32f407xx.s

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=stm32f4discovery-buspirate

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
AR=arm-none-eabi-ar
OBJCOPY=arm-none-eabi-objcopy

ODIR=obj

C_DEFS := -DUSE_HAL_DRIVER -DSTM32F407xx

FLAGS += -O2 -Wall $(C_DEFS)
FLAGS += --specs=nosys.specs -mthumb -mcpu=cortex-m4
FLAGS += -Tstm32f4discovery.ld -fdata-sections -ffunction-sections -Wl,--gc-sections
CFLAGS = --std=gnu11  $(FLAGS)
CPPFLAGS = --std=gnu++11 $(FLAGS)

# Include files from STM libraries
INCLUDE += -I$(BUSPIRATE_DIR)
INCLUDE += -I$(INC_DIR)
INCLUDE += -I$(CMSIS_CORE)
INCLUDE += -I$(CMSIS_STM32)/Include
INCLUDE += -I$(STM_LIB_SRC)/Inc
INCLUDE += -I$(STM_USB_CORE)/Inc
INCLUDE += -I$(STM_USB_CDC)/Inc

STM_SRC_EXP := $(wildcard $(STM_SRC))
STM_OBJ := $(STM_SRC_EXP:.c=.o)
STM_OBJ += $(STARTUP:.s=.o)

BUSPIRATE_SRC_EXP := $(wildcard $(BUSPIRATE_SRC))
BUSPIRATE_OBJ     := $(BUSPIRATE_SRC_EXP:.c=.o)

SRC_EXP := $(wildcard $(USR_SRC))
SRC_OBJ := $(SRC_EXP:.c=.o)

.PHONY: clean all size dfu

.c.o:
	$(CC) -c $(INCLUDE) $(CFLAGS)  $< -o $@

.cpp.o:
	$(CXX) -c $(INCLUDE) $(CPPFLAGS)  $< -o $@

.s.o:
	$(CC) -c $(INCLUDE) $(CFLAGS)  $< -o $@

all: main size

main: $(BUSPIRATE_OBJ) $(STM_OBJ) $(SRC_OBJ)  main.o
	$(CC) $(CFLAGS) $(INCLUDE) $(SRC_OBJ) $(BUSPIRATE_OBJ) $(STM_OBJ) main.o -o $(PROJ_NAME).elf
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex

clean:
	rm -f *.o $(CAN_OBJ) $(STM_OBJ) $(SRC_OBJ) $(PROJ_NAME)*.elf $(PROJ_NAME)*.bin $(PROJ_NAME)*.bin

size: 
	arm-none-eabi-size $(PROJ_NAME)*.elf

tags: force_look
	ctags -R *

force_look:
	true

# Flash the STM32F4
flash: all
	dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000:leave -D $(PROJ_NAME).bin

stflash: main
	st-flash write $(PROJ_NAME).bin 0x08000000