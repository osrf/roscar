BIN=bin
OUT=$(BIN)/$(NAME)
TC=arm-none-eabi-
OPENOCD=openocd -f ../common/st_nucleo_f7.cfg -f ../common/stm32f7x.cfg
COMMON_SRCS=stack stm32f7_vectors startup stubs led pin systime console delay flash usb
#COMMON_SRCS=startup stubs led pin systime console delay usb dmxl
COMMON_OBJS=$(COMMON_SRCS:%=$(BIN)/%.o)
OBJS=$(SRCS:%.c=$(BIN)/%.o)
IMAGE_START ?= 0x08000000
ARCH_FLAGS=-mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16
CFLAGS=-g $(ARCH_FLAGS) -ffunction-sections -fdata-sections -MD -std=gnu99 -Wall -I../common -DSTM32F746 -DHSE_VALUE=8000000 -Werror -I.
LDFLAGS=-g $(ARCH_FLAGS)

default: $(BIN) $(OUT).bin

$(BIN):
	mkdir -p $(BIN)

$(COMMON_OBJS): $(BIN)/%.o: ../common/%.c
	$(TC)gcc $(CFLAGS) -c $< -o $@

$(BIN)/%.o: %.c
	$(TC)gcc $(CFLAGS) -c $< -o $@

$(OUT): $(OBJS) $(COMMON_OBJS)
	$(TC)gcc $(OBJS) $(COMMON_OBJS) -lc -lgcc -lm -T ../common/stm32f746.ld -Wl,--no-gc-sections $(LDFLAGS) -o $(OUT) -Wl,-Map=$(BIN)/$(NAME).map,--cref
	$(TC)objdump -S -d $(OUT) > $(OUT).objdump

$(OUT).bin: $(OUT)
	$(TC)objcopy -O binary $(OUT) $(OUT).bin
	$(TC)size $(OUT)

clean:
	-rm -rf $(BIN)

unlock:
	$(OPENOCD) -c "init; halt; flash info 0; stm32f2x unlock 0; flash protect 0 0 last off; shutdown"

info:
	$(OPENOCD) -c "init; halt; flash banks; flash info 0; stm32f2x options_read 0; shutdown"

erase:
	$(OPENOCD) -c "init; halt; stm32f2x unlock 0; reset halt; stm32f2x mass_erase 0; shutdown"

program: $(OUT).bin
	$(OPENOCD) -c "init; sleep 100; halt; sleep 100; flash write_image erase $(OUT).bin $(IMAGE_START); verify_image $(OUT).bin $(IMAGE_START); sleep 100; reset run; sleep 100; shutdown"

dump_flash: $(BIN)
	$(OPENOCD) -c "init; halt; flash banks; dump_image $(OUT).bin.dump $(IMAGE_START) 0x1000; reset run; shutdown"

gdb_server: $(OUT).bin
	$(OPENOCD) -c "init; halt"

gdb: $(OUT).bin
	$(TC)gdb $(OUT) -x ../common/gdb_init_commands

reset:
	$(OPENOCD) -c "init; reset halt; sleep 100; halt; sleep 100; reset run; sleep 100; shutdown"
