# Standalone build alongside CubeIDE managed Debug/Release configurations.
# Set TOOLCHAIN=/path/to/bin when arm-none-eabi-gcc is not on PATH.
TOOLCHAIN ?= $(patsubst %/arm-none-eabi-gcc,%,$(shell command -v arm-none-eabi-gcc 2>/dev/null))
ifeq ($(TOOLCHAIN),)
TOOLCHAIN := $(firstword $(wildcard /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin))
endif
PREFIX := $(if $(TOOLCHAIN),$(TOOLCHAIN)/)arm-none-eabi-
CC := $(PREFIX)gcc
OBJDUMP := $(PREFIX)objdump
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size
BUILD ?= build
NAME := Buck_Voltage_Mode
DIAGNOSTICS ?= 0
SCOPE_PIN ?= 0
AUTO_START ?= 0
USE_HSE ?= 0
ESTIMATOR ?= 1
EST_DISPLAY ?= 1
OPTIMAL ?= 0
SAMPLE_LEAD ?= 144
CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := -DSTM32G474xx -IInc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
            -DBUCK_DIAGNOSTICS=$(DIAGNOSTICS) -DBUCK_SCOPE_PIN=$(SCOPE_PIN) \
            -DBUCK_AUTO_START=$(AUTO_START) -DBUCK_USE_HSE=$(USE_HSE) \
            -DBUCK_ESTIMATOR=$(ESTIMATOR) -DBUCK_EST_DISPLAY=$(EST_DISPLAY) -DBUCK_OPTIMAL=$(OPTIMAL) -DBUCK_SAMPLE_LEAD_CYCLES=$(SAMPLE_LEAD)
CFLAGS := $(CPU) -std=gnu11 -O3 -g3 -ffunction-sections -fdata-sections -fno-common \
          -fno-builtin -Wall -Wextra -Wshadow -Wdouble-promotion -MMD -MP
LDSCRIPT ?= STM32G474RETX_FLASH.ld
LDFLAGS := $(CPU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections,-Map=$(BUILD)/$(NAME).map -Wl,--print-memory-usage
SOURCES := $(wildcard Src/*.c)
OBJECTS := $(patsubst Src/%.c,$(BUILD)/%.o,$(SOURCES)) $(BUILD)/startup.o

.PHONY: all clean test check FORCE
all: $(BUILD)/$(NAME).elf $(BUILD)/$(NAME).hex $(BUILD)/$(NAME).bin $(BUILD)/$(NAME).list
$(BUILD):
	mkdir -p $@
$(BUILD)/config: FORCE | $(BUILD)
	@python3 tools/build_config.py $@ '$(CC)' '$(CPPFLAGS)' '$(CFLAGS)' '$(LDFLAGS)'
$(BUILD)/%.o: Src/%.c $(BUILD)/config | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
$(BUILD)/startup.o: Startup/startup_stm32g474retx.s $(BUILD)/config | $(BUILD)
	$(CC) $(CPU) -g3 -c $< -o $@
$(BUILD)/$(NAME).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@
$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@
$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@
$(BUILD)/%.list: $(BUILD)/%.elf
	$(OBJDUMP) -d -S $< > $@
test: | $(BUILD)
	cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=undefined $(CPPFLAGS) Tests/test_pi.c -o $(BUILD)/test_pi
	$(BUILD)/test_pi
	cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=undefined $(CPPFLAGS) Tests/test_estimator.c -lm -o $(BUILD)/test_estimator
	$(BUILD)/test_estimator
	cc -std=c11 -O2 -Wall -Wextra -Werror $(CPPFLAGS) Tests/test_optimal.c -lm -o $(BUILD)/test_optimal
	$(BUILD)/test_optimal
	cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=undefined $(CPPFLAGS) Tests/test_cm.c -lm -o $(BUILD)/test_cm
	$(BUILD)/test_cm
	cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=undefined $(CPPFLAGS) Tests/test_solve.c -lm -o $(BUILD)/test_solve
	$(BUILD)/test_solve
check: all test
	python3 tools/check_elf.py '$(PREFIX)' $(BUILD)/$(NAME).elf
clean:
	rm -rf $(BUILD)
-include $(OBJECTS:.o=.d)
