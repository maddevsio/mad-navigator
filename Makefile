ARM_TOOLCHAIN_PREFIX = arm-none-eabi-
CC=$(ARM_TOOLCHAIN_PREFIX)gcc
AS=$(ARM_TOOLCHAIN_PREFIX)as
LD=$(ARM_TOOLCHAIN_PREFIX)ld
OBJDUMP=$(ARM_TOOLCHAIN_PREFIX)objdump
OBJCOPY=$(ARM_TOOLCHAIN_PREFIX)objcopy
SIZE=$(ARM_TOOLCHAIN_PREFIX)size

BUILD_DIR=build
BIN_DIR=bin
CMSIS_DIR=CMSIS
STD_PERIPH_DRIVER_DIR=StdPeriph_Driver

LIBS = -lm
DEFS = -DSTM32 -DSTM32F1 -DSTM32F100RBTx -DSTM32F10X_MD_VL -DUSE_STDPERIPH_DRIVER -DBEARING_HIGH_PRECISION
WARN_LEVEL = -Wall

#device and program
PRG = kov
MMCU = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft 
OPTIMIZE = -O0 -g3
INCLUDES = -Iinc -Iinc/gps -Iinc/accelerometer -Iinc/magnetometer -Iinc/nokiaLcd -I$(CMSIS_DIR)/core -I$(CMSIS_DIR)/device -I$(STD_PERIPH_DRIVER_DIR)/inc

CFLAGS = $(INCLUDES) $(MMCU) $(OPTIMIZE) $(DEFS) $(WARN_LEVEL) -fmessage-length=0 -ffunction-sections -MMD -MP

LDFLAGS = $(MMCU) -Wl,-T,LinkerScript.ld -Wl,-Map,$(BIN_DIR)/$(PRG).map -Wl,--gc-sections -Wl,-print-memory-usage 

OSRC0=$(wildcard *.c) $(wildcard src/*.c) $(wildcard src/accelerometer/*.c) $(wildcard src/accelerometer/*/*.c)
OSRC1=$(wildcard src/*/*.c) $(wildcard src/display/*.c) $(wildcard src/display/*/*.c) $(wildcard src/magnetometer/*.c) $(wildcard src/magnetometer/*/*.c)
OSRC2=$(wildcard $(CMSIS_DIR)/*/*.c)
OSRC3=$(wildcard $(STD_PERIPH_DRIVER_DIR)/*/*.c)
OSRC4=$(wildcard startup/*.s)

OBJECTS=$(patsubst %.c, %.o, $(OSRC0) $(OSRC1) $(OSRC2) $(OSRC3)) $(patsubst %.s, %.o, $(OSRC4))

all: directories $(PRG)
startup/startup_stm32.o: startup/startup_stm32.s
	$(AS) $(INCLUDES) $(MMCU) -g -o $@ $^

$(PRG): $(BIN_DIR)/$(PRG).elf $(BIN_DIR)/lst $(BIN_DIR)/$(PRG).bin

$(BIN_DIR)/$(PRG).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

$(BIN_DIR)/lst: $(BIN_DIR)/$(PRG).lst
$(BIN_DIR)/%.lst: $(BIN_DIR)/%.elf
	$(OBJDUMP) -h -S $< > $@

$(BIN_DIR)/bin: $(BIN_DIR)/$(PRG).bin
$(BIN_DIR)/%.bin: $(BIN_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

clean:
	@rm -rf $(BUILD_DIR)/*
	@rm -rf $(BIN_DIR)/*
	@rm -rf CMSIS/core/*.o CMSIS/core/*.d
	@rm -rf StdPeriph_Driver/src/*.o StdPeriph_Driver/src/*.d
	@rm -rf src/commons/*.o src/commons/*.d
	@rm -rf src/display/*.o src/display/*.d
	@rm -rf src/magnetometer/*.o src/magnetometer/lis3mdl/*.o src/magnetometer/hmc5883l/*.o src/magnetometer/qmc5883l/*.o
	@rm -rf src/accelerometer/*.o src/accelerometer/mpu6050/*.o
	@rm -rf src/gps/*.o src/gps/*.d
	@rm -rf src/power/*.o src/power/*.d
	@rm -rf src/time/*.o src/time/*.d
	@rm -rf startup/*.o

mrproper:
	@rm -rf $(BUILD_DIR)
	@rm -rf $(BIN_DIR)

program:
	@st-flash write $(BIN_DIR)/$(PRG).bin 0x8000000

flash:
	@st-flash write $(BIN_DIR)/$(PRG).bin 0x8000000
