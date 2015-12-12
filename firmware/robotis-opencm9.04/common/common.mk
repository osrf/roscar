BIN=bin
OUT=$(BIN)/$(NAME)
TC=arm-none-eabi-
OPENOCD=openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/stm32f1x.cfg -f ../common/robotis-opencm.cfg 
#COMMON_SRCS=usb enc parity
COMMON_SRCS=startup stubs led pin systime console delay usb
COMMON_OBJS=$(COMMON_SRCS:%=$(BIN)/%.o)
OBJS=$(SRCS:%.c=$(BIN)/%.o)
IMAGE_START ?= 0x08000000

CFLAGS=-g -mthumb -mcpu=cortex-m3 -ffunction-sections -fdata-sections -MD -std=gnu99 -Wall -I../common -DSTM32F103C8 -DHSE_VALUE=8000000 -Werror -I.
LDFLAGS=-g $(HARD_FLOAT_FLAGS) -mthumb -mcpu=cortex-m3

default: $(BIN) $(OUT).bin

$(BIN):
	mkdir -p $(BIN)

$(COMMON_OBJS): $(BIN)/%.o: ../common/%.c
	$(TC)gcc $(CFLAGS) -c $< -o $@

$(BIN)/%.o: %.c
	$(TC)gcc $(CFLAGS) -c $< -o $@

$(OUT): $(OBJS) $(COMMON_OBJS)
	$(TC)gcc $(OBJS) $(COMMON_OBJS) -lc -lgcc -lm -T ../common/stm32f103c8.ld -Wl,--no-gc-sections $(LDFLAGS) -o $(OUT) -Wl,-Map=$(BIN)/$(NAME).map,--cref
	$(TC)objdump -S -d $(OUT) > $(OUT).objdump

$(OUT).bin: $(OUT)
	$(TC)objcopy -O binary $(OUT) $(OUT).bin
	$(TC)size $(OUT)

clean:
	-rm -rf $(BIN)

unlock:
	$(OPENOCD) -c "init; halt; flash info 0; stm32f1x unlock 0; flash protect 0 0 last off; shutdown"

info:
	$(OPENOCD) -c "init; halt; flash banks; flash info 0; stm32f1x options_read 0; shutdown"

erase:
	$(OPENOCD) -c "init; halt; stm32f1x unlock 0; reset halt; stm32f1x mass_erase 0; shutdown"

program: $(OUT).bin
	$(OPENOCD) -c "init; sleep 100; halt; sleep 100; flash write_image $(OUT).bin $(IMAGE_START); verify_image $(OUT).bin $(IMAGE_START); sleep 100; reset run; sleep 100; shutdown"

dump_flash: $(BIN)
	$(OPENOCD) -c "init; halt; flash banks; dump_image $(OUT).bin.dump $(IMAGE_START) 0x1000; reset run; shutdown"

gdb_server: $(OUT).bin
	$(OPENOCD) -c "init; halt"

gdb: $(OUT).bin
	$(TC)gdb $(OUT) -x $(ROOT)/common/stm32/gdb_init_commands

reset:
	$(OPENOCD) -c "init; reset halt; sleep 100; halt; sleep 100; reset run; sleep 100; shutdown"
