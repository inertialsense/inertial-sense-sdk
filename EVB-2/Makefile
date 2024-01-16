# MIT LICENSE

# Copyright 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

TARGET	?= EVB-2
IS_TARGET = $(addprefix IS_,$(TARGET))

DEBUG ?=

#################################
# GNU ARM Embedded Toolchain
#################################

ifeq ($(OS),Windows_NT) # Windows (any, not just NT)

# Windows mkdir isn't same as linux
mkdir = mkdir $(subst /,\,$(1)) > nul 2>&1 || (exit 0)
rm = rmdir /Q /S $(subst /,\,$(1)) > nul 2>&1 || (exit 0)

COMPILER_DIR := $(subst \,/,C:/Inertial_Sense/compilers/arm-none-eabi-stm32/tools/bin)
ifeq ("$(wildcard $(COMPILER_DIR)/*)","")
COMPILER_DIR := $(subst \,/,C:/PROGRA~2/Atmel/Studio/7.0/toolchain/arm/arm-gnu-toolchain/bin)
endif
ifeq ("$(wildcard $(COMPILER_DIR)/*)","")
$(error Windows: $(wildcard $(COMPILER_DIR)/*) compiler does not exist)	
endif

CC = $(addsuffix /arm-none-eabi-gcc.exe,$(COMPILER_DIR))
CXX = $(addsuffix /arm-none-eabi-g++.exe,$(COMPILER_DIR))
LD = $(addsuffix /arm-none-eabi-ld.exe,$(COMPILER_DIR))
CP = $(addsuffix /arm-none-eabi-objcopy.exe,$(COMPILER_DIR))
OD = $(addsuffix /arm-none-eabi-objdump.exe,$(COMPILER_DIR))
SIZE = $(addsuffix /arm-none-eabi-size.exe,$(COMPILER_DIR))

else # Linux

mkdir = mkdir -p $(1)
rm = rm -f -r $(1)

# Detect if a compiler exists.
COMPILER_DIR := /opt/arm-none-eabi-uins3/bin
ifeq ("$(wildcard $(COMPILER_DIR)/*)","")
# Use the old arm-none-eabi from before we split arm-none-eabi-uins3/arm-none-eabi-stm32
COMPILER_DIR = /opt/arm-none-eabi/bin
endif
ifeq ("$(wildcard $(COMPILER_DIR)/*)","")
$(error Linux: $(wildcard $(COMPILER_DIR)/*) compiler does not exist)	
endif

CC=$(COMPILER_DIR)/arm-none-eabi-gcc
CXX=$(COMPILER_DIR)/arm-none-eabi-g++
LD=$(COMPILER_DIR)/arm-none-eabi-ld
AR=$(COMPILER_DIR)/arm-none-eabi-ar
AS=$(COMPILER_DIR)/arm-none-eabi-as
CP=$(COMPILER_DIR)/arm-none-eabi-objcopy
OD=$(COMPILER_DIR)/arm-none-eabi-objdump
NM=$(COMPILER_DIR)/arm-none-eabi-nm
SIZE=$(COMPILER_DIR)/arm-none-eabi-size
A2L=$(COMPILER_DIR)/arm-none-eabi-addr2line

endif # OS

#################################
# Working directories
#################################
ROOT          = .
SRC_DIR       = $(ROOT)/IS_EVB-2/src
SDK_DIR       = ..
HW_LIBS_DIR   = ../hw-libs
ASF_DIR       = $(HW_LIBS_DIR)/drivers/SAMx70/ASF

#################################
# Debug/Release selection
#################################
ifeq ($(DEBUG), GDB)
BUILD_MODE = debug
DEBUG_FLAGS = -g3
OPT_FLAGS = -O0
DEFS += -DDEBUG
OBJECT_DIR = Debug/cpp/hdw-src
BIN_DIR = Debug
else
BUILD_MODE = release
DEBUG_FLAGS = -g3
OPT_FLAGS = -O3
DEFS += -DNDEBUG
OBJECT_DIR = Release/cpp/hdw-src
BIN_DIR = Release
endif

#################################
# Source Files
#################################

C_SRCS += \
	$(SDK_DIR)/src/com_manager.c                                                                   \
	$(SDK_DIR)/src/convert_ins.cpp                                                                 \
	$(SDK_DIR)/src/data_sets.c                                                                     \
	$(SDK_DIR)/src/DeviceLog.cpp                                                                   \
	$(SDK_DIR)/src/DeviceLogSerial.cpp                                                             \
	$(SDK_DIR)/src/DataChunk.cpp                                                                   \
	$(SDK_DIR)/src/filters.cpp                                                                     \
	$(SDK_DIR)/src/ISComm.c                                                                        \
	$(SDK_DIR)/src/ISDataMappings.cpp                                                              \
	$(SDK_DIR)/src/ISEarth.c                                                                       \
	$(SDK_DIR)/src/ISFileManager.cpp                                                               \
	$(SDK_DIR)/src/ISLogger.cpp                                                                    \
	$(SDK_DIR)/src/ISLogStats.cpp                                                                  \
	$(SDK_DIR)/src/ISMatrix.c                                                                      \
	$(SDK_DIR)/src/ISPose.cpp                                                                      \
	$(SDK_DIR)/src/ISUtilities.cpp                                                                 \
	$(SDK_DIR)/src/protocol_nmea.cpp                                                               \
	$(ASF_DIR)/sam/utils/syscalls/gcc/syscalls.c                                                   \
	$(ASF_DIR)/common/components/memory/sd_mmc/sd_mmc.c                                            \
	$(ASF_DIR)/common/components/memory/sd_mmc/sd_mmc_mem.c                                        \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/common/source/nm_common.c                  \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/m2m_wifi.c                   \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/bsp/source/nm_bsp_same70.c                 \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/socket/source/socket.c                     \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/m2m_hif.c                    \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmbus.c                      \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmspi.c                      \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/bus_wrapper/source/nm_bus_wrapper_same70.c \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmasic.c                     \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmdrv.c                      \
	$(ASF_DIR)/common/services/clock/same70/sysclk.c                                               \
	$(ASF_DIR)/common/services/delay/sam/cycle_counter.c                                           \
	$(ASF_DIR)/common/services/serial/usart_serial.c                                               \
	$(ASF_DIR)/common/services/sleepmgr/sam/sleepmgr.c                                             \
	$(ASF_DIR)/common/services/spi/sam_spi/spi_master.c                                            \
	$(ASF_DIR)/common/services/spi/sam_usart_spi/usart_spi.c                                       \
	$(ASF_DIR)/common/services/storage/ctrl_access/ctrl_access.c                                   \
	$(ASF_DIR)/common/services/usb/class/cdc/device/udi_cdc.c                                      \
	$(ASF_DIR)/common/services/usb/class/cdc/device/udi_cdc_desc.c                                 \
	$(ASF_DIR)/common/services/usb/udc/udc.c                                                       \
	$(ASF_DIR)/common/utils/interrupt/interrupt_sam_nvic.c                                         \
	$(ASF_DIR)/common/utils/stdio/read.c                                                           \
	$(ASF_DIR)/common/utils/stdio/write.c                                                          \
	$(ASF_DIR)/sam/drivers/afec/afec.c                                                             \
	$(ASF_DIR)/sam/drivers/efc/efc.c                                                               \
	$(ASF_DIR)/sam/drivers/hsmci/hsmci.c                                                           \
	$(ASF_DIR)/sam/drivers/mcan/mcan.c                                                             \
	$(ASF_DIR)/sam/drivers/mpu/mpu.c                                                               \
	$(ASF_DIR)/sam/drivers/pio/pio.c                                                               \
	$(ASF_DIR)/sam/drivers/pio/pio_handler.c                                                       \
	$(ASF_DIR)/sam/drivers/pmc/pmc.c                                                               \
	$(ASF_DIR)/sam/drivers/pmc/sleep.c                                                             \
	$(ASF_DIR)/sam/drivers/pwm/pwm.c                                                               \
	$(ASF_DIR)/sam/drivers/rtc/rtc.c                                                               \
	$(ASF_DIR)/sam/drivers/rtt/rtt.c                                                               \
	$(ASF_DIR)/sam/drivers/spi/spi.c                                                               \
	$(ASF_DIR)/sam/drivers/tc/tc.c                                                                 \
	$(ASF_DIR)/sam/drivers/twihs/twihs.c                                                           \
	$(ASF_DIR)/sam/drivers/uart/uart.c                                                             \
	$(ASF_DIR)/sam/drivers/usart/usart.c                                                           \
	$(ASF_DIR)/sam/drivers/usbhs/usbhs_device.c                                                    \
	$(ASF_DIR)/sam/drivers/wdt/wdt.c                                                               \
	$(ASF_DIR)/sam/drivers/xdmac/xdmac.c                                                           \
	$(ASF_DIR)/sam/services/flash_efc/flash_efc.c                                                  \
	$(ASF_DIR)/sam/utils/cmsis/same70/source/templates/gcc/startup_same70.c                        \
	$(ASF_DIR)/sam/utils/cmsis/same70/source/templates/system_same70.c                             \
	$(ASF_DIR)/thirdparty/fatfs/fatfs-port-r0.09/diskio.c                                          \
	$(ASF_DIR)/thirdparty/fatfs/fatfs-port-r0.09/sam/fattime_rtc.c                                 \
	$(ASF_DIR)/thirdparty/fatfs/fatfs-r0.09/src/ff.c                                               \
	$(ASF_DIR)/thirdparty/fatfs/fatfs-r0.09/src/option/ccsbcs.c                                    \
	$(HW_LIBS_DIR)/misc/new.cpp                                                                    \
	$(HW_LIBS_DIR)/drivers/SAMx70/CAN.c                                                            \
	$(HW_LIBS_DIR)/drivers/SAMx70/d_dma.c                                                          \
	$(HW_LIBS_DIR)/drivers/SAMx70/d_flash.c                                                        \
	$(HW_LIBS_DIR)/drivers/SAMx70/d_i2c.c                                                          \
	$(HW_LIBS_DIR)/drivers/SAMx70/d_serial.c                                                       \
	$(HW_LIBS_DIR)/drivers/SAMx70/d_time.c                                                         \
	$(HW_LIBS_DIR)/bootloader/bootloaderApp.c                                                      \
	$(HW_LIBS_DIR)/misc/rtos.c                                                                     \
	$(HW_LIBS_DIR)/printf/printf.c                                                                 \
	$(HW_LIBS_DIR)/freertos/croutine.c                                                             \
	$(HW_LIBS_DIR)/freertos/event_groups.c                                                         \
	$(HW_LIBS_DIR)/freertos/portable/MemMang/heap_4.c                                              \
	$(HW_LIBS_DIR)/freertos/list.c                                                                 \
	$(HW_LIBS_DIR)/freertos/portable/GCC/ARM_CM7/r0p1/port.c                                       \
	$(HW_LIBS_DIR)/freertos/queue.c                                                                \
	$(HW_LIBS_DIR)/freertos/tasks.c                                                                \
	$(HW_LIBS_DIR)/freertos/timers.c                                                               \
	$(HW_LIBS_DIR)/freertos/stream_buffer.c                                                        \
	$(SRC_DIR)/communications.cpp                                                                  \
	$(SRC_DIR)/evb_tasks.cpp                                                                       \
	$(SRC_DIR)/globals.c                                                                           \
	$(SRC_DIR)/ISLogFileFatFs.cpp                                                                  \
	$(SRC_DIR)/main.cpp                                                                            \
	$(SRC_DIR)/sd_card_logger.cpp                                                                  \
	$(SRC_DIR)/spiTouINS.c                                                                         \
	$(SRC_DIR)/user_interface.cpp                                                                  \
	$(SRC_DIR)/wheel_encoder.cpp                                                                   \
	$(SRC_DIR)/wifi.c                                                                              \
	$(SRC_DIR)/xbee.cpp                                                                            \
	$(SRC_DIR)/drivers/d_quadEnc.c                                                                 \
	$(SRC_DIR)/drivers/d_adc.c                                                                     \
	$(SRC_DIR)/misc/init.c

#################################
# Include Directories
#################################

INCLUDES +=	\
	$(SRC_DIR) \
	$(SRC_DIR)/config \
	$(SRC_DIR)/drivers \
	$(SRC_DIR)/misc

INCLUDES += $(SDK_DIR)/src

INCLUDES += \
	$(HW_LIBS_DIR)                	                                                \
	$(HW_LIBS_DIR)/boards 			                                                \
	$(HW_LIBS_DIR)/bootloader 		                                                \
	$(HW_LIBS_DIR)/communications 	                                                \
	$(HW_LIBS_DIR)/drivers/SAMx70                                                   \
	$(HW_LIBS_DIR)/drivers/common                                                   \
	$(HW_LIBS_DIR)/printf                                                           \
	$(HW_LIBS_DIR)/misc  

INCLUDES += \
	$(ASF_DIR)/common/boards                                                        \
	$(ASF_DIR)/common/components/memory/sd_mmc                                      \
	$(ASF_DIR)/common/components/wifi/winc3400/wifi_drv                             \
	$(ASF_DIR)/common/services/clock                                                \
	$(ASF_DIR)/common/services/delay                                                \
	$(ASF_DIR)/common/services/gpio                                                 \
	$(ASF_DIR)/common/services/ioport                                               \
	$(ASF_DIR)/common/services/serial/sam_uart                                      \
	$(ASF_DIR)/common/services/serial                                               \
	$(ASF_DIR)/common/services/sleepmgr                                             \
	$(ASF_DIR)/common/services/spi/sam_spi                                          \
	$(ASF_DIR)/common/services/spi/sam_usart_spi                                    \
	$(ASF_DIR)/common/services/spi                                                  \
	$(ASF_DIR)/common/services/storage/ctrl_access                                  \
	$(ASF_DIR)/common/services/usb/class/cdc/device                                 \
	$(ASF_DIR)/common/services/usb/class/cdc                                        \
	$(ASF_DIR)/common/services/usb/udc                                              \
	$(ASF_DIR)/common/services/usb                                                  \
	$(ASF_DIR)/common/utils/stdio/stdio_serial                                      \
	$(ASF_DIR)/common/utils                                                         \
	$(ASF_DIR)/sam/boards                                                           \
	$(ASF_DIR)/sam/drivers/efc                                                      \
	$(ASF_DIR)/sam/drivers/hsmci                                                    \
	$(ASF_DIR)/sam/drivers/mcan                                                     \
	$(ASF_DIR)/sam/drivers/mpu                                                      \
	$(ASF_DIR)/sam/drivers/pio                                                      \
	$(ASF_DIR)/sam/drivers/pmc                                                      \
	$(ASF_DIR)/sam/drivers/pwm                                                      \
	$(ASF_DIR)/sam/drivers/rtc                                                      \
	$(ASF_DIR)/sam/drivers/rtt                                                      \
	$(ASF_DIR)/sam/drivers/spi                                                      \
	$(ASF_DIR)/sam/drivers/tc                                                       \
	$(ASF_DIR)/sam/drivers/twihs                                                    \
	$(ASF_DIR)/sam/drivers/uart                                                     \
	$(ASF_DIR)/sam/drivers/usart                                                    \
	$(ASF_DIR)/sam/drivers/usbhs                                                    \
	$(ASF_DIR)/sam/drivers/wdt                                                      \
	$(ASF_DIR)/sam/drivers/afec                                                     \
	$(ASF_DIR)/sam/drivers/xdmac                                                    \
	$(ASF_DIR)/sam/services/flash_efc                                               \
	$(ASF_DIR)/sam/utils/cmsis/same70/include                                       \
	$(ASF_DIR)/sam/utils/cmsis/same70/include/component                             \
	$(ASF_DIR)/sam/utils/cmsis/same70/source/templates                              \
	$(ASF_DIR)/sam/utils/fpu                                                        \
	$(ASF_DIR)/sam/utils/header_files                                               \
	$(ASF_DIR)/sam/utils/preprocessor                                               \
	$(ASF_DIR)/sam/utils                                                            \
	$(ASF_DIR)/thirdparty/CMSIS/Include                                             \
	$(ASF_DIR)/thirdparty/CMSIS/Lib/GCC                                             \
	$(ASF_DIR)/thirdparty/fatfs/fatfs-port-r0.09/sam                                \
	$(ASF_DIR)/thirdparty/fatfs/fatfs-r0.09/src                                     \
	$(HW_LIBS_DIR)/freertos/include \
	$(HW_LIBS_DIR)/freertos/portable/GCC/ARM_CM7/r0p1 \

LDSCRIPT=$(ROOT)/IS_EVB-2/SAMx70-FLASH.ld

#################################
# Object List
#################################
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(C_SRCS))))
DEPENDENCIES+=$(patsubst %.o,%.d,$(OBJECTS))

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(IS_TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(IS_TARGET).hex
TARGET_BIN=$(BIN_DIR)/$(IS_TARGET).bin
TARGET_LSS=$(BIN_DIR)/$(IS_TARGET).lss
TARGET_MAP=$(BIN_DIR)/$(IS_TARGET).map

#################################
# Flags
#################################

DEFS += \
	-D__SAME70Q20B__ \
	-DARM_MATH_CM7=true \
	-D__FREERTOS__ \
	-DUDD_ENABLE \
	-DSD_MMC_ENABLE \
	-DARM \
	-D__INERTIAL_SENSE_EVB_2__ \
	-DCONF_PERIPH \
	-DEVB_2 \

# CPU type flags
CPU_FLAGS = \
	-mcpu=cortex-m7 \
	-mthumb \
	-mfloat-abi=softfp \
	-mfpu=fpv5-d16 \

# Dependency flags
DEP_FLAGS = -MD -MP -MF $(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o)

# C assorted flags
FLAGS_C = -x c -c -pipe -std=gnu11 -fdata-sections -ffunction-sections \
	-fno-strict-aliasing --param max-inline-insns-single=500 
# C defines, CPU flags
FLAGS_C += $(CPU_FLAGS) $(DEFS) -Dscanf=iscanf
# C includes
FLAGS_C += $(addprefix -I,$(INCLUDES)) 
# C optimization and debug flags
FLAGS_C += $(OPT_FLAGS) $(DEBUG_FLAGS)
# C warning flags
FLAGS_C += \
	-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations \
	-Wimplicit-function-declaration -Wpointer-arith -Wchar-subscripts -Wno-comment \
	-Wformat=2 -Wimplicit-int -Wmain -Wparentheses -Wsequence-point -Wreturn-type \
	-Wswitch -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wundef -Wshadow \
	-Wwrite-strings -Wsign-compare -Wmissing-declarations -Wformat -Wmissing-format-attribute \
	-Wpacked -Wunreachable-code -Wcast-align -Wno-unused -Wno-unused-function \
	-Wno-misleading-indentation -Wno-int-conversion
# C dependency flags
FLAGS_C += $(DEP_FLAGS)

# C++ defines, CPU flags
FLAGS_CXX = $(CPU_FLAGS) $(DEFS)
# C++ includes
FLAGS_CXX += $(addprefix -I,$(INCLUDES))
# C++ optimization and debug flags
FLAGS_CXX += $(OPT_FLAGS) $(DEBUG_FLAGS)
# C++ assorted flags
FLAGS_CXX += -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections \
		-fno-rtti -fno-exceptions -c -std=gnu++11 
# C++ warning flags
FLAGS_CXX += -Wno-comment -Wno-switch -Wno-unused-function -Wno-parentheses -Wno-psabi
# C++ dependency flags
FLAGS_CXX += $(DEP_FLAGS)

FLAGS_LD = \
	-mthumb -Wl,-Map="$(TARGET_MAP)" -Wl,--start-group -lm \
	-Wl,--end-group --specs=nano.specs -Wl,--gc-sections -Wl,--entry=Reset_Handler \
	-Wl,--cref -mthumb -T$(LDSCRIPT) $(CPU_FLAGS) -L"../src/ASF/thirdparty/CMSIS/Lib/GCC"

FLAGS_HEX = -O ihex -R .eeprom -R .fuse -R .lock -R .signature

#################################
# Recipes
#################################
.PHONY: all clean

all: main-build
	
pre-build:
	@$(call update_info)
	$(info Compiler found.)
	$(info Using $(CC) as C compiler)
	$(info Using $(CXX) as C++ compiler)
	$(info Using $(LD) as linker)
	$(info Building in $(BUILD_MODE) mode)

main-build: pre-build
	@$(MAKE) --no-print-directory $(TARGET_BIN) $(TARGET_HEX)

clean:
	$(info Cleaning project)
	@$(call rm,$(BIN_DIR))

-include $(DEPENDENCIES)

#################################
# Build
#################################

$(TARGET_BIN): $(TARGET_ELF)
	$(info Creating $@)
	@$(CP) -O binary $< $@
	
$(TARGET_HEX): $(TARGET_ELF)
	$(info Creating $@)
	@$(CP) $(FLAGS_HEX) $< $@
	
$(TARGET_ELF): $(OBJECTS)
	$(info Linking $@ in $(BUILD_MODE) mode...)
	@$(CXX) -o $@ $(OBJECTS) $(FLAGS_LD)  
	@$(OD) -h -S $@ > $(TARGET_LSS)
	$(info Program size table:)
	@$(SIZE) $@

$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@$(call mkdir,$(dir $@))
	$(info $(notdir $<))
	@$(CXX) $(FLAGS_CXX) -o $@ $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@$(call mkdir,$(dir $@))
	$(info $(notdir $<))
	@$(CC) $(FLAGS_C) -o $@ $<
