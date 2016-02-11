#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
# Toolchain Paths
#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
TARGET_CHIP := NRF51822_QFAA_G0
BOARD := BOARD_PCA10001
DEVICE := NRF51
DEVICESERIES := nrf51

SDK_PATH = ../../../

GNU_INSTALL_ROOT := /usr#$(SDK_PATH)gcc-arm-none-eabi-4_8-2014q2
GNU_VERSION = 4.8.4
GNU_PREFIX = arm-none-eabi

C_SOURCE_FILES = main.c system_nrf51.c
C_SOURCE_FILES += usart.c
C_SOURCE_FILES += misc.c
C_SOURCE_FILES += ble_phy.c
C_SOURCE_FILES += sensor.c
C_SOURCE_FILES += spi.c
C_SOURCE_FILES += delay.c
C_SOURCE_FILES += led.c 
C_SOURCE_FILES += dfu_app_handler.c


C_SOURCE_FILES += $(SDK_PATH)components/ble/common/ble_srv_common.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/ble_error_log/ble_error_log.c
C_SOURCE_FILES += $(SDK_PATH)ble/common/ble_advdata.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/ble_advertising/ble_advertising.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/ble_radio_notification/ble_radio_notification.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/ble_debug_assert_handler/ble_debug_assert_handler.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/common/ble_conn_params.c
#C_SOURCE_FILES += $(SDK_PATH)Source/ble/ble_bondmngr.c

C_SOURCE_FILES += $(SDK_PATH)components/softdevice/common/softdevice_handler/softdevice_handler.c 
C_SOURCE_FILES += $(SDK_PATH)components/drivers_nrf/pstorage/pstorage.c
#C_SOURCE_FILES += $(SDK_PATH)Source/app_common/crc16.c
C_SOURCE_FILES += $(SDK_PATH)components/drivers_nrf/hal/nrf_delay.c
C_SOURCE_FILES += $(SDK_PATH)components/libraries/scheduler/app_scheduler.c 
C_SOURCE_FILES += $(SDK_PATH)components/libraries/timer/app_timer.c 
C_SOURCE_FILES += $(SDK_PATH)components/libraries/timer/app_timer_appsh.c 
C_SOURCE_FILES += $(SDK_PATH)components/drivers_nrf/gpiote/nrf_drv_gpiote.c
C_SOURCE_FILES += $(SDK_PATH)components/drivers_nrf/common/nrf_drv_common.c 



C_SOURCE_FILES += $(SDK_PATH)components/libraries/bootloader_dfu/bootloader_util.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/ble_services/ble_dfu/ble_dfu.c
#C_SOURCE_FILES += $(SDK_PATH)components/libraries/bootloader_dfu/dfu_app_handler.c
C_SOURCE_FILES += $(SDK_PATH)components/ble/device_manager/device_manager_peripheral.c

#$(abspath ../../../../../../components/libraries/fifo/app_fifo.c) \
#$(abspath ../../../../../../components/ble/common/ble_conn_params.c) \
#$(abspath ../../../../../../components/ble/ble_services/ble_dfu/ble_dfu.c) \


#assembly files common to all targets
SOFTDEVICE = ./softdevice/s110_softdevice.bin
SOFTDEVICE_DIRECTORY = ./softdevice
OUTPUT_FILENAME := main
SDK_INCLUDE_PATH = $(SDK_PATH)/Include/
SDK_SOURCE_PATH = $(SDK_PATH)Source/
TEMPLATE_PATH += $(SDK_PATH)templates/gcc/
OUTPUT_BINARY_DIRECTORY := build
#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
# Toolchain commands
#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
CPU := cortex-m0

CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
SIZE       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"

MK 				:= mkdir
RM 				:= rm -rf

OBJECT_DIRECTORY := $(OUTPUT_BINARY_DIRECTORY)
LISTING_DIRECTORY := $(OUTPUT_BINARY_DIRECTORY)

#C_SOURCE_FILES += system_$(DEVICESERIES).c
ASSEMBLER_SOURCE_FILES += gcc_startup_$(DEVICESERIES).s

#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
# Linker, compailer & assembler flags
#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
LDFLAGS += -L"$(GNU_INSTALL_ROOT)/arm-none-eabi/lib/armv6-m"
LDFLAGS += -L"$(GNU_INSTALL_ROOT)/lib/gcc/arm-none-eabi/$(GNU_VERSION)/armv6-m"
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mcpu=$(CPU) -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -Tgcc_nrf51_s110_xxaa.ld 

CFLAGS += -mcpu=$(CPU) -mthumb -mabi=aapcs -D$(DEVICE) -DBLE_STACK_SUPPORT_REQD -D$(BOARD) -D$(TARGET_CHIP) --std=gnu99
CFLAGS += -Wall -DSWI_DISABLE0 -DSWI_DISABLE3
CFLAGS += -mfloat-abi=soft 
CFLAGS += -DSOFTDEVICE_PRESENT 
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBLE_DFU_APP_SUPPORT

ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBLE_DFU_APP_SUPPORT

INCLUDEPATHS  = -I./
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/util
INCLUDEPATHS += -I$(SDK_PATH)components/toolchain/gcc
INCLUDEPATHS += -I$(SDK_PATH)components/toolchain
INCLUDEPATHS += -I$(SDK_PATH)components/ble/common
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_error_log
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_debug_assert_handler
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/scheduler
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/gpiote
INCLUDEPATHS += -I$(SDK_PATH)components/softdevice/s110/headers
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/timer
INCLUDEPATHS += -I../../../../../bsp
INCLUDEPATHS += -I$(SDK_PATH)components/device
INCLUDEPATHS += -I$(SDK_PATH)components/softdevice/common/softdevice_handler
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/hal
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/ble_flash
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/gpiote
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/config
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/common
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/pstorage
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/button
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/scheduler




INCLUDEPATHS += -I$(SDK_PATH)components/libraries/util
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_advertising
INCLUDEPATHS += -I$(SDK_PATH)components/ble/common
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_advertising
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_services/ble_hids
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_services/ble_bas
INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_services/ble_dis
INCLUDEPATHS += -I$(SDK_PATH)components/ble/device_manager
INCLUDEPATHS += -I$(SDK_PATH)components/ble/device_manager/config
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/sensorsim
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/timer
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/gpiote
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/button
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/trace
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/util
INCLUDEPATHS += -I$(SDK_PATH)components/softdevice/s110/headers
INCLUDEPATHS += -I$(SDK_PATH)components/softdevice/common/softdevice_handler
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/hal
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/pstorage
INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/pstorage/config


INCLUDEPATHS += -I$(SDK_PATH)components/ble/ble_services/ble_dfu
INCLUDEPATHS += -I$(SDK_PATH)components/libraries/bootloader_dfu
INCLUDEPATHS += -I$(SDK_PATH)components/ble/device_manager
#INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/gpiote/
#INCLUDEPATHS += -I$(SDK_PATH)components/drivers_nrf/common/

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )
#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
# Rules                                                            
#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww

C_SOURCE_FILENAMES = $(notdir $(C_SOURCE_FILES) )
ASSEMBLER_SOURCE_FILENAMES = $(notdir $(ASSEMBLER_SOURCE_FILES) )

# Make a list of source paths -> solves the issue of the coping *.c & *.o files to build
C_SOURCE_PATHS = ./ ../ $(SDK_SOURCE_PATH) $(TEMPLATE_PATH) $(wildcard $(SDK_SOURCE_PATH)*/)  
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)*/)
C_SOURCE_PATHS += $(wildcard  ./*/)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/libraries/*/)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/ble/*/)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/ble/ble_services/*/)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/drivers_nrf/*)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/drivers_nrf/gpiote/*/)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/softdevice/common/*/)
C_SOURCE_PATHS += $(wildcard  $(SDK_PATH)components/softdevice/s110/*/)
#C_SOURCE_PATHS += $(SDK_PATH)components/softdevice/common/softdevice_handler/softdevice_handler.c

ASSEMBLER_SOURCE_PATHS = ../ $(SDK_SOURCE_PATH) $(TEMPLATE_PATH) $(wildcard $(SDK_SOURCE_PATH)*/)

C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/,$(C_SOURCE_FILENAMES:.c=.o))
ASSEMBLER_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/,$(ASSEMBLER_SOURCE_FILENAMES:.s=.o) )

# Set source lookup paths
vpath %.c $(C_SOURCE_PATHS)
vpath %.s $(ASSEMBLER_SOURCE_PATHS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJECT_DIRECTORY)/, $(COBJS:.o=.d))

### Targets
debug:    CFLAGS += -DDEBUG -g3 -O2
debug:    ASMFLAGS += -DDEBUG -g3 -O2
debug:    $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

.PHONY: release
release: clean
release:  CFLAGS += -DNDEBUG -O2
release:  ASMFLAGS += -DNDEBUG -O2
release:  $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echostuff:
	@echo C_OBJECTS: [$(C_OBJECTS)]
	@echo C_SOURCE_FILES: [$(C_SOURCE_FILES)]

## Create build directories
$(BUILD_DIRECTORIES):
	$(MK) $@

## Create objects from C source files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	@$(CC) $(CFLAGS) $(INCLUDEPATHS) -c -o $@ $<

## Assemble .s files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	@$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

## Link C and assembler objects to an .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(C_OBJECTS) $(ASSEMBLER_OBJECTS) $(LIBRARIES)
	$(CC) $(LDFLAGS) $(C_OBJECTS) $(ASSEMBLER_OBJECTS) $(LIBRARIES) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(SIZE) -B  $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	
	
OPENOCD	:= $(SDK_PATH)openocd/src/openocd -f interface/stlink-v2.cfg -c"transport select hla_swd" -f target/nrf51.cfg 


upload-app: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin
	$(OPENOCD) -c "program $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin  0x00018000  verify; reset"
	
upload-dfu: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin
	$(OPENOCD) -c "program dfu/dfu.bin  0x0003A000  verify; reset"
	
upload-dfu-uicr: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin
	$(OPENOCD) -c "program dfu/dfu-uicr2.bin  0x10001000  verify; reset"

upload-softdevice: $(SOFTDEVICE) 
	$(OPENOCD) -c "init; reset halt; nrf51 mass_erase ; sleep 500 ;flash write_image $(SOFTDEVICE) 0x0 ; verify_image $(SOFTDEVICE) 0; shutdown"

mob-app:
	./crc-ccitt
	rm main.zip
	zip -xi -j main.zip  ./$(OUTPUT_BINARY_DIRECTORY)/main.bin $(OUTPUT_BINARY_DIRECTORY)/manifest.json $(OUTPUT_BINARY_DIRECTORY)/main.dat 
 
#$(OPENOCD) -c "init; reset halt; nrf51 mass_erase ; sleep 500 ;flash write_image dfu/dfu-uicr.bin 0x10001014 ; verify_image dfu/dfu-uicr.bin 0x10001014 ; flash write_image $(SOFTDEVICE) 0x0 ; verify_image $(SOFTDEVICE) 0; shutdown"
	

hex-upload: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(OPENOCD)   "flash write_image ble.hex  0 ihex  verify; reset"

##upload-softdevice: $(SOFTDEVICE_DIRECTORY)/s110_softdevice_mainpart.bin $(SOFTDEVICE_DIRECTORY)/s110_softdevice_uicr.bin
##$(OPENOCD) -c "init; reset halt; nrf51 mass_erase ; sleep 500 ; flash write_image $(SOFTDEVICE_DIRECTORY)/s110_softdevice_uicr.bin 0x10001000 ; verify_image $(SOFTDEVICE_DIRECTORY)/s110_softdevice_uicr.bin 0x10001000 ; flash write_image $(SOFTDEVICE_DIRECTORY)/s110_softdevice_mainpart.bin 0x0 ; verify_image $(SOFTDEVICE_DIRECTORY)/s110_softdevice_mainpart.bin 0; shutdown"

merge:
	srec_cat softdevice/s110_softdevice.hex -intel dfu/dfu.hex -intel -o ble.hex -intel --line-length=44
	srec_cat ble.hex -intel build/main.hex -intel -o ble.hex -intel --line-length=44

genbin:
	$(OBJCOPY) -I ihex -Obinary ble.hex  build/main.bin
erase_all:
	$(OPENOCD) -c "init ; reset halt ; nrf51 mass_erase ; shutdown"

pinreset:
	$(OPENOCD) -c "init ; reset halt ; mww 0x4001e504 2 ; mww 0x40000544 1 ; reset ; shutdown"

clean:
	rm -rf $(OUTPUT_BINARY_DIRECTORY)/

gdbserver:
	$(OPENOCD)

GDB_COMMAND_SCRIPT=$(OUTPUT_BINARY_DIRECTORY)/gdbinit
$(GDB_COMMAND_SCRIPT): debug
	echo -e -n "target remote localhost:3333    \n\
        monitor reset halt                \n\
        file $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out  \n\
        load                            \n\
        b main                          \n\
        b app_error_handler             \n\
        monitor reset                   \n\
        continue" > $(GDB_COMMAND_SCRIPT)

flash: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin
	$(OPENOCD) -c "program $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin  0x00000000  verify; reset"
