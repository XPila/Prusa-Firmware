#-----------------------------------------------------------------------------------------------------------------------
# Makefile to compile and link Prusa-Firmware
PROJECT ?= Prusa-Firmware

# build configuration list for make
ALL_CONFIGURATIONS  := Debug Release
# selected build configuration, default Release
BUILD_CONFIGURATION  ?= Release
# check valid build configuration
ifneq (,$(filter-out $(ALL_CONFIGURATIONS), $(BUILD_CONFIGURATION)))
$(error invalid configuration "$(BUILD_CONFIGURATION)")
endif

PRN_VARIANT ?= 0

# target platform prefix, default is avr
TOOLCHAIN_PREFIX ?= avr

# file path separator (default is unix '/', for windows use - '\\')
PATH_SEPARATOR ?= /
# unix shell or windows shell - determine by PATH_SEPARATOR
SHELL_IS_UNIX ?= $(findstring /,$(PATH_SEPARATOR))
# shell command for removing directories
SHELL_RMDIR ?= $(if $(SHELL_IS_UNIX),rm -rvf,RMDIR /S /Q)
# shell command for creating directories
SHELL_MKDIR ?= $(if $(SHELL_IS_UNIX),mkdir -p,MKDIR)
# shell command for removing files
SHELL_RM ?= $(if $(SHELL_IS_UNIX),rm,DEL /Q)
# shell command for renaming files/dirs
SHELL_MV ?= $(if $(SHELL_IS_UNIX),mv,REN)
# shell command 'cat'
SHELL_CAT ?= $(if $(SHELL_IS_UNIX),cat,TYPE)
# shell command separator
SHELL_CMDSEP ?= $(if $(SHELL_IS_UNIX), ; , & )

# microcontroler chip type
MCU ?= atmega2560
# cpu frequency [Hz]
F_CPU ?= 16000000

# arduino version (means 1.8.5)
ARDUINO ?= 10805
# arduino core path
ARDUINO_PATH ?= c:\arduino-1.8.5\hardware\arduino\avr\cores\arduino
# arduino board variant path
#VARIANT_PATH ?= c:\arduino-1.8.5\hardware\arduino\avr\variants\mega
VARIANT_PATH ?= c:\arduino-1.8.5\hardware\marlin\avr\variants\rambo

# avrdude
AVRDUDE ?= .\bin\avrdude

# output folder (output directory tree will be automaticaly created)
OUT := build/$(BUILD_CONFIGURATION)
OUT_ARDUINO := $(OUT)/Arduino

# gcc, g++ and objcopy tool
GCC    := $(TOOLCHAIN_PREFIX)-gcc
GPP    := $(TOOLCHAIN_PREFIX)-g++
OBJCPY := $(TOOLCHAIN_PREFIX)-objcopy

# output files
OUTDIR := $(OUT)/$(PROJECT).dir # file containing list of created directories
OUTELF := $(OUT)/$(PROJECT).elf # elf output file
OUTHEX := $(OUT)/$(PROJECT).hex # hex output file
OUTBIN := $(OUT)/$(PROJECT).bin # bin output file

# all source
ALLSRC := \
$(addprefix Firmware/,adc.c AutoDeplete.cpp BlinkM.cpp bootapp.c cardreader.cpp cmdqueue.cpp\
	ConfigurationStore.cpp conv2str.cpp Dcodes.cpp fsensor.cpp language.c lcd.cpp Marlin_main.cpp MarlinSerial.cpp\
	menu.cpp mesh_bed_calibration.cpp mesh_bed_leveling.cpp messages.c mmu.cpp motion_control.cpp\
	optiboot_w25x20cl.cpp pat9125.c planner.cpp qr_solve.cpp rbuf.c Sd2Card.cpp SdBaseFile.cpp SdFatUtil.cpp\
	SdFile.cpp SdVolume.cpp Servo.cpp sm4.c sound.cpp spi.c stepper.cpp swi2c.c swspi.cpp temperature.cpp Timer.cpp\
	timer02.c tmc2130.cpp uart2.c ultralcd.cpp util.cpp vector_3.cpp w25x20cl.c xyzcal.cpp)# Configuration.cpp eeprom.cpp first_lay_cal.cpp) 

# arduino source
ARDUINOSRC := \
$(addprefix $(ARDUINO_PATH)/,hooks.c wiring.c wiring_digital.c wiring_analog.c main.cpp Tone.cpp) 

# external symbol definitions
SYMBOLS := \
	F_CPU=$(F_CPU)UL\
	ARDUINO=$(ARDUINO)\
	ARDUINO_AVR_RAMBO\
	ARDUINO_ARCH_AVR\
	PRN_VARIANT=$(PRN_VARIANT)

# include directories
INCLUDES := 

# common flags
CMNFLAGS := -mmcu=$(MCU) -g -flto

# compiler flags
#original gcc flags: -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10805 -DARDUINO_AVR_RAMBO -DARDUINO_ARCH_AVR
#original g++ flags: -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10805 -DARDUINO_AVR_RAMBO -DARDUINO_ARCH_AVR
ASMFLAGS := $(CMNFLAGS)
GCCFLAGS := $(CMNFLAGS) -ffunction-sections -fdata-sections -Wall -std=gnu99 -MMD -fno-fat-lto-objects
GPPFLAGS := $(CMNFLAGS) -ffunction-sections -fdata-sections -Wall -std=gnu++11 -fpermissive -fno-exceptions
OPTIMIZE := -Os
ifneq (,$(findstring Debug, $(BUILD_CONFIGURATION)))
	SYMBOLS += _DEBUG
	OPTIMIZE := -O0
endif
GCCFLAGS += $(OPTIMIZE) $(addprefix -D,$(SYMBOLS)) $(addprefix -I./,$(INCLUDES)) -I$(ARDUINO_PATH) -I$(VARIANT_PATH)
GPPFLAGS += $(OPTIMIZE) $(addprefix -D,$(SYMBOLS)) $(addprefix -I./,$(INCLUDES)) -I$(ARDUINO_PATH) -I$(VARIANT_PATH)

# linker flags
# original linker flags: -w -Os -Wl,-u,vfprintf -lprintf_flt -lm -Wl,--gc-sections,--relax -mmcu=atmega2560 
LDFLAGS  := $(CMNFLAGS) -Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_min \
	-Wl,--gc-sections -Wl,-Map="$(OUT)/$(PROJECT).map",--cref

# list of all directories
ALLDIR := $(addprefix $(OUT)/,$(sort $(subst / , ,$(filter-out ./,$(dir $(ALLSRC))) )))

# lists of all assembler, C and C++ source files
ASMSRC := $(filter %.s, $(ALLSRC))
GCCSRC := $(filter %.c, $(ALLSRC))
GPPSRC := $(filter %.cpp, $(ALLSRC))
ARDUINO_GCCSRC := $(filter %.c, $(ARDUINOSRC))
ARDUINO_GPPSRC := $(filter %.cpp, $(ARDUINOSRC))

# lists of all assembler, C and C++ object files
ASMOBJ := $(addprefix $(OUT)/,$(ASMSRC:.s=.o))
GCCOBJ := $(addprefix $(OUT)/,$(GCCSRC:.c=.o))
GPPOBJ := $(addprefix $(OUT)/,$(GPPSRC:.cpp=.o))
ARDUINO_GCCOBJ := $(addprefix $(OUT)/Arduino,$(subst $(ARDUINO_PATH),,$(ARDUINO_GCCSRC:.c=.o)))
ARDUINO_GPPOBJ := $(addprefix $(OUT)/Arduino,$(subst $(ARDUINO_PATH),,$(ARDUINO_GPPSRC:.cpp=.o)))


all: build

build:
	@make -s $(OUTELF) $(OUTHEX) $(OUTBIN)

$(OUTDIR):
	@echo creating output directory tree
	@echo $(subst /,$(PATH_SEPARATOR),$(OUT))
	@$(SHELL_MKDIR) $(subst /,$(PATH_SEPARATOR),$(OUT))
	@$(SHELL_MKDIR) $(subst /,$(PATH_SEPARATOR),$(OUT)/Arduino)
ifneq ("$(ALLDIR)","")
	@$(SHELL_MKDIR) $(subst /,$(PATH_SEPARATOR),$(ALLDIR))
endif
	@echo $(ALLDIR) >$(OUTDIR)

$(OUT)/%.o: %.s $(OUTDIR)
	@echo compiling $<
	@$(GCC) -c $(ASMFLAGS) -o $@ $<

$(OUT)/%.o: %.c $(OUTDIR)
	@echo compiling $<
	@$(GCC) -c $(GCCFLAGS) -o $@ $<

$(OUT)/%.o: %.cpp $(OUTDIR)
	@echo compiling $<
	@$(GPP) -c $< $(GPPFLAGS) -o $@

$(OUT_ARDUINO)/%.o: $(ARDUINO_PATH)/%.c $(OUTDIR)
	@echo compiling arduino $<
	@$(GCC) -c $(GCCFLAGS) -o $@ $<

$(OUT_ARDUINO)/%.o: $(ARDUINO_PATH)/%.cpp $(OUTDIR)
	@echo compiling arduino $<
	@$(GPP) -c $< $(GPPFLAGS) -o $@

$(OUTELF): $(ASMOBJ) $(GCCOBJ) $(GPPOBJ) $(ARDUINO_GCCOBJ) $(ARDUINO_GPPOBJ)
	@echo linking $@
	@$(file > $(OUT)/.objlist,$(ASMOBJ) $(GCCOBJ) $(GPPOBJ) $(ARDUINO_GCCOBJ) $(ARDUINO_GPPOBJ))
	@$(GPP) -o $(OUTELF) @$(OUT)/.objlist $(LDFLAGS)

$(OUTHEX): $(OUTELF)
	@echo generating hex file $@
	@$(OBJCPY) -O ihex $< $@

$(OUTBIN): $(OUTELF)
	@echo generating bin file $@
	@$(OBJCPY) -O binary $< $@

clean:
	@echo removing output files $(OUT)
ifneq ("$(wildcard $(OUT))","")
	@$(SHELL_RMDIR) $(subst /,$(PATH_SEPARATOR),$(OUT))
endif

flash:
	@echo flashing hexfile $(OUTHEX) using avrdude
#	@$(AVRDUDE) -p atmega8 -P usb -c usbasp -F -v -u -U flash:w:$(subst /,$(PATH_SEPARATOR),$(OUTHEX)):i
	@$(AVRDUDE) -V -v -v -p atmega2560 -c wiring -P COM12 -b115200 -D -U flash:w:$(subst /,$(PATH_SEPARATOR),$(OUTHEX)):i
#	@$(AVRDUDE) -C./bin/avrdude.conf -V -v -v -p atmega2560 -c wiring -P $port -b115200 -D -U flash:w:./output/Firmware.ino.hex:i 

build_MK34_MK3:
	@make -s build PRN_VARIANT=1

build_MK34_trapez_MK3:
	@make -s build PRN_VARIANT=2

build_MK34_trapez_X:
	@make -s build PRN_VARIANT=4

.PHONY: all build clean


test:
#	@echo $(ARDUINO_GCCOBJ)
	@make -s build PRN_VARIANT=1
	