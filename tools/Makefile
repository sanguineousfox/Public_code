# Reproducible build independent of CubeIDE's Windows-generated Debug files.
# make -f tools/Makefile CROSS=/path/to/arm-none-eabi-
CROSS ?= arm-none-eabi-
CC := $(CROSS)gcc
ROOT := MagStrain
OUT ?= build-fw
FW_MINOR ?= 0
INC := -I$(ROOT)/Core/Inc -I$(ROOT)/Drivers/STM32F1xx_HAL_Driver/Inc -I$(ROOT)/Drivers/CMSIS/Include -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx/Include
FLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32F103xB -DUSE_HAL_DRIVER $(INC) -Os -g3 -ffunction-sections -fdata-sections -Wall -Wextra
HAL := $(wildcard $(ROOT)/Drivers/STM32F1xx_HAL_Driver/Src/*.c)
START := $(ROOT)/Core/Startup/startup_stm32f103c8tx.s
APP := $(wildcard $(ROOT)/Core/Src/*.c) $(HAL) $(START)
BOOT := $(ROOT)/Bootloader/main.c $(ROOT)/Core/Src/fw_update.c $(ROOT)/Core/Src/i2c_config.c $(ROOT)/Core/Src/AT24C64.c $(ROOT)/Core/Src/system_stm32f1xx.c $(HAL) $(START)
LINK := -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -Wl,--start-group -lc -lm -Wl,--end-group
.PHONY: all clean standalone
all: $(OUT)/application.bin $(OUT)/bootloader.bin
standalone: $(OUT)/standalone.elf
$(OUT)/standalone.elf: $(APP) $(wildcard $(ROOT)/Core/Inc/*.h) $(ROOT)/STM32F103C8TX_FLASH.ld | $(OUT)
	$(CC) $(FLAGS) $(APP) -T$(ROOT)/STM32F103C8TX_FLASH.ld -Wl,-Map=$(OUT)/standalone.map $(LINK) -o $@
	$(CROSS)size $@
$(OUT):
	mkdir -p $@
$(OUT)/application.elf: $(APP) $(wildcard $(ROOT)/Core/Inc/*.h) $(ROOT)/Bootloader/application.ld $(ROOT)/Bootloader/debug_boot_image.inc | $(OUT)
	$(CC) $(FLAGS) $(CPPFLAGS) -DFIRMWARE_VERSION_MINOR=$(FW_MINOR) $(APP) -T$(ROOT)/Bootloader/application.ld -Wl,-Map=$(OUT)/application.map $(LINK) -o $@
	$(CROSS)size $@
$(OUT)/bootloader.elf: $(BOOT) $(wildcard $(ROOT)/Core/Inc/*.h) $(ROOT)/Bootloader/boot.ld | $(OUT)
	$(CC) $(FLAGS) -DFW_BOOTLOADER $(BOOT) -T$(ROOT)/Bootloader/boot.ld -Wl,-Map=$(OUT)/bootloader.map $(LINK) -o $@
	$(CROSS)size $@
$(OUT)/%.bin: $(OUT)/%.elf
	$(CROSS)objcopy -O binary $< $@
clean:
	rm -rf $(OUT)

.PHONY: test
test:
	python3 tests/check_compatibility.py
	gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -Itests -I$(ROOT)/Core/Inc tests/test_fw_update.c $(ROOT)/Core/Src/fw_update.c -o /tmp/magstrain-test-fw
	ASAN_OPTIONS=detect_leaks=0 /tmp/magstrain-test-fw
	gcc -std=c11 -Wall -Wextra -Werror -Wno-int-to-pointer-cast -DFW_BOOTLOADER -Itests -I$(ROOT)/Core/Inc tests/test_installer.c $(ROOT)/Core/Src/fw_update.c -o /tmp/magstrain-test-installer
	/tmp/magstrain-test-installer
	gcc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,float-cast-overflow -DUSART2_DEBUG_ENABLED=0 -Itests -I$(ROOT)/Core/Inc tests/test_graduation.c $(ROOT)/Core/Src/graduation.c -lm -o /tmp/magstrain-test-graduation
	/tmp/magstrain-test-graduation
	gcc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Itests -I$(ROOT)/Core/Inc tests/test_sensor_calibration_grid.c $(ROOT)/Core/Src/sensor_calibration_grid.c $(ROOT)/Core/Src/measurement_statistics.c -lm -o /tmp/magstrain-test-sensor-calibration
	/tmp/magstrain-test-sensor-calibration
	gcc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -I$(ROOT)/Core/Inc tests/test_liquid_properties.c $(ROOT)/Core/Src/liquid_properties.c $(ROOT)/Core/Src/measurement_snapshot.c -lm -o /tmp/magstrain-test-liquid-properties
	/tmp/magstrain-test-liquid-properties
	python3 -m py_compile tools/modbus_update.py
