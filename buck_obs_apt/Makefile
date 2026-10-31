# Standalone build alongside the CubeIDE managed Debug/Release configurations.
# Set TOOLCHAIN=/path/to/bin when arm-none-eabi-gcc is not on PATH.
TOOLCHAIN ?= $(patsubst %/arm-none-eabi-gcc,%,$(shell command -v arm-none-eabi-gcc 2>/dev/null))
ifeq ($(TOOLCHAIN),)
TOOLCHAIN := $(firstword $(wildcard /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin))
endif
PREFIX  := $(if $(TOOLCHAIN),$(TOOLCHAIN)/)arm-none-eabi-
CC      := $(PREFIX)gcc
OBJDUMP := $(PREFIX)objdump
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
BUILD   ?= build
NAME    := buck_obs_apt
CPU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := -DSTM32G474xx -IInc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32G4xx/Include
CFLAGS  := $(CPU) -std=gnu11 -O3 -g3 -ffunction-sections -fdata-sections -fno-common \
           -fno-builtin -ffp-contract=off -Wall -Wextra -Wshadow -Wdouble-promotion -MMD -MP
LDSCRIPT ?= STM32G474RETX_FLASH.ld
LDFLAGS := $(CPU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections,-Map=$(BUILD)/$(NAME).map -Wl,--print-memory-usage
# The TOC math is header-only since stage 4 (Inc/buck_toc.h, also built for the host
# tests); the TOC firmware (Src/buck_toc.c) is linked since stage 2 (29 Oct 2026).
TOC_SRC :=
SOURCES := $(wildcard Src/*.c)
OBJECTS := $(patsubst Src/%.c,$(BUILD)/%.o,$(SOURCES)) $(BUILD)/startup.o
HOSTCC  ?= cc

.PHONY: all clean test check toc-test
all: $(BUILD)/$(NAME).elf $(BUILD)/$(NAME).hex $(BUILD)/$(NAME).bin $(BUILD)/$(NAME).list
$(BUILD):
	mkdir -p $@
$(BUILD)/%.o: Src/%.c Makefile | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
$(BUILD)/startup.o: Startup/startup_stm32g474retx.s | $(BUILD)
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
# Host tests: the per-cycle algorithm (Inc/buck_ctrl.h) against a double-
# precision transcription of the specification, and a closed-loop simulation
# of the switched converter driven by the same code.
test: | $(BUILD)
	$(HOSTCC) -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off -fsanitize=undefined -fno-sanitize-recover=undefined -IInc Tests/test_ctrl.c -lm -o $(BUILD)/test_ctrl
	$(BUILD)/test_ctrl
# Host tests of the feed-forward TOC (stage 4): the L-C solve and the Jacobian step
# (Inc/buck_toc.h) against double-precision L-C integrations, and the frame's sequence
# on the switched plant (Tests/test_toc.c).
toc-test: | $(BUILD)
	$(HOSTCC) -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off -fsanitize=undefined -fno-sanitize-recover=undefined -IInc Tests/test_toc.c $(TOC_SRC) -lm -o $(BUILD)/test_toc
	$(BUILD)/test_toc
# Post-link checks of the control loop's compiled form (tools/check_loop.py).
NM      := $(PREFIX)nm
check: $(BUILD)/$(NAME).elf
	python3 tools/check_loop.py $(OBJDUMP) $(NM) $<
clean:
	rm -rf $(BUILD)
-include $(OBJECTS:.o=.d)
