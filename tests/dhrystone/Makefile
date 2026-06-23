BUILD_DIR := build

UNAME_S := $(shell uname -s)
ifeq ($(origin CROSS),undefined)
ifeq ($(UNAME_S),Darwin)
CROSS := i686-elf-
else
CROSS :=
endif
endif

CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
HOST_32BIT_FLAG := $(if $(CROSS),,-m32)

DHRY_ITERS ?= 200
OPT ?= -O2

CFLAGS := $(HOST_32BIT_FLAG) -march=i386 -mno-80387 -ffreestanding -fno-builtin
CFLAGS += -fno-pic -fno-pie -fno-stack-protector -fno-asynchronous-unwind-tables
CFLAGS += -std=gnu89 -Wall -Wno-implicit-int -Wno-implicit-function-declaration
CFLAGS += -Wno-return-type -Wno-builtin-declaration-mismatch
CFLAGS += $(OPT) -DREG=register -DDHRY_EMBEDDED -DDHRY_ITERS=$(DHRY_ITERS)

LDFLAGS := $(HOST_32BIT_FLAG) -nostdlib -static -no-pie -Wl,-T,dhrystone.ld
LDFLAGS += -Wl,-Map,$(BUILD_DIR)/dhrystone.map

SRCS := startup.S dhrystone_main.c dhry_1.c dhry_2.c support.c
OBJS := $(addprefix $(BUILD_DIR)/,$(SRCS:.c=.o))
OBJS := $(OBJS:.S=.o)

.PHONY: all clean force run compare

all: $(BUILD_DIR)/dhrystone.bin $(BUILD_DIR)/dhrystone.lst

run: compare

compare:
	./run_dhrystone.py --iters $(DHRY_ITERS)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/%.o: %.c force | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.S force | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/dhrystone.elf: $(OBJS) dhrystone.ld | $(BUILD_DIR)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(BUILD_DIR)/dhrystone.bin: $(BUILD_DIR)/dhrystone.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/dhrystone.lst: $(BUILD_DIR)/dhrystone.elf
	$(OBJDUMP) -drwC -Mintel $< > $@

clean:
	rm -rf $(BUILD_DIR)
